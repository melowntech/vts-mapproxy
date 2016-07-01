#include <sys/types.h>

#include <new>
#include <array>
#include <atomic>
#include <thread>

#include <boost/noncopyable.hpp>
#include <boost/format.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "geo/gdal.hpp"

#include "../error.hpp"
#include "../gdalsupport.hpp"
#include "./process.hpp"
#include "./datasetcache.hpp"
#include "./types.hpp"
#include "./operations.hpp"

namespace asio = boost::asio;
namespace bs = boost::system;

namespace {

typedef boost::posix_time::ptime SystemTime;

SystemTime systemTime()
{
    // we must have miscrosec clock to work properly
    return boost::date_time::microsec_clock<SystemTime>::universal_time();
}

template <typename Delta>
SystemTime absTime(const Delta &delta)
{
    return systemTime() + delta;
}

typedef boost::posix_time::milliseconds milliseconds;

class ShRasterRequest : boost::noncopyable {
public:
    typedef bi::deleter<ShRasterRequest, SegmentManager> Deleter;
    typedef bi::shared_ptr<ShRasterRequest, Allocator, Deleter> pointer;
    typedef bi::weak_ptr<ShRasterRequest, Allocator, Deleter> wpointer;

    typedef bi::deque<pointer, bi::allocator<pointer, SegmentManager>> Deque;

    ShRasterRequest(const GdalWarper::RasterRequest &other
                    , ManagedBuffer &sm)
        : sm_(sm)
        , operation_(other.operation)
        , dataset_(other.dataset.data()
                 , other.dataset.size()
                 , sm.get_allocator<char>())
        , srs_(other.srs.srs.data()
               , other.srs.srs.size()
               , sm.get_allocator<char>())
        , srsType_(other.srs.type)
        , extents_(other.extents)
        , size_(other.size)
        , resampling_(other.resampling)
        , mask_(sm.get_allocator<char>())
        , done_(false)
        , response_()
        , error_(sm.get_allocator<char>())
    {
        if (other.mask) {
            mask_.assign(other.mask->data(), other.mask->size());
        }
    }

    ~ShRasterRequest() {
        if (response_) {
            sm_.deallocate(response_);
        }
    }

    operator GdalWarper::RasterRequest() const {
        return GdalWarper::RasterRequest
            (operation_
             , std::string(dataset_.data(), dataset_.size())
             , geo::SrsDefinition(asString(srs_), srsType_)
             , extents_, size_, resampling_
             , asOptional(mask_));
    }

    template <typename T>
    void setError(bi::interprocess_mutex &mutex, const T &what);

    void setError(Lock&, const char *message);
    void setError(Lock&, const std::exception &e);
    void setError(Lock&, const GenerateError &e);

    GdalWarper::Raster get(Lock &lock);
    GdalWarper::Raster get(bi::interprocess_mutex &mutex);
    void set(bi::interprocess_mutex &mutex, cv::Mat *response);

    static pointer create(const GdalWarper::RasterRequest &req
                          , ManagedBuffer &mb)
    {
        return pointer(mb.construct<ShRasterRequest>
                       (bi::anonymous_instance)(req, mb)
                       , mb.get_allocator<void>()
                       , mb.get_deleter<ShRasterRequest>());
    }

private:
    std::string asString(const String &str) const {
        return { str.data(), str.size() };
    }

    boost::optional<std::string> asOptional(const String &str) const {
        if (str.empty()) { return {}; }
        return std::string(str.data(), str.size());
    }

    ManagedBuffer &sm_;

    GdalWarper::RasterRequest::Operation operation_;
    String dataset_;
    String srs_;
    geo::SrsDefinition::Type srsType_;
    math::Extents2 extents_;
    math::Size2 size_;
    geo::GeoDataset::Resampling resampling_;
    String mask_;

    // response condition and flag
    bi::interprocess_condition cond_;
    bool done_;

    // response matrix
    cv::Mat *response_;

    // response error
    String error_;
    GenerateError::RaiseError raiseError_;
};

template <typename T>
void ShRasterRequest::setError(bi::interprocess_mutex &mutex, const T &what)
{
    Lock lock(mutex);
    setError(lock, what);
}

void ShRasterRequest::setError(Lock&, const char *message)
{
    if (!error_.empty()) { return; }
    error_.assign(message);
    raiseError_ = &GenerateError::runtimeError;
    done_ = true;
    cond_.notify_one();
}

void ShRasterRequest::setError(Lock&, const GenerateError &e)
{
    if (!error_.empty()) { return; }
    error_.assign(e.what());
    raiseError_ = e.getRaise();
    done_ = true;
    cond_.notify_one();
}

void ShRasterRequest::setError(Lock &lock, const std::exception &e)
{
    setError(lock, e.what());
}

void ShRasterRequest::set(bi::interprocess_mutex &mutex, cv::Mat *response)
{
    Lock lock(mutex);
    if (response_) { return; }
    response_ = response;
    done_ = true;
    cond_.notify_one();
}

GdalWarper::Raster ShRasterRequest::get(Lock &lock)
{
    cond_.wait(lock, [&]()
    {
        return done_;
    });

    if (response_) {
        // steal and wrap in std::smard_ptr
        auto response(response_);
        response_ = 0;
        return GdalWarper::Raster(response, [&sm_](cv::Mat *mat)
        {
            // deallocate data
            sm_.deallocate(mat);
        });
    }

    if (raiseError_) {
        raiseError_(std::string(error_.data(), error_.size()));
    }

    throw std::runtime_error("Unknown exception!");
}

GdalWarper::Raster ShRasterRequest::get(bi::interprocess_mutex &mutex)
{
    Lock lock(mutex);
    return get(lock);
}

struct Worker {
    typedef bi::deleter<Worker, SegmentManager> Deleter;
    typedef bi::shared_ptr<Worker, Allocator, Deleter> pointer;
    typedef std::map<Process::Id, pointer> map;

    Worker() {}

    void attach(Process &&process) { process_ = std::move(process); }

    void join(bool justTry = false) {
        if (process_.joinable()) {
            process_.join(justTry);
        }
    }

    void kill() {
        // kill the process, ignore all errors
        if (process_.joinable()) {
            try {
                process_.kill();
            } catch(...) {}
        }
    }

    void associate(const ShRasterRequest::pointer &req) { req_ = req; }

    void disassociate() { req_ = {}; }

    Process::Id id() const { return process_.id(); }

    void internalError(bi::interprocess_mutex &mutex)
    {
        Lock lock(mutex);
        if (req_) {
            req_->setError(lock, InternalError
                           ("GDAL warper process unexpectedly terminated"));
        }
    }

    static pointer create(ManagedBuffer &mb) {
        return pointer(mb.construct<Worker>(bi::anonymous_instance)()
                       , mb.get_allocator<void>()
                       , mb.get_deleter<Worker>());
    }

private:
    /** Worker process.
     */
    Process process_;

    /** Processed request.
     */
    ShRasterRequest::pointer req_;
};

} // namespace

class GdalWarper::Detail
{
public:
    Detail(const Options &options);
    ~Detail();

    Raster warp(const RasterRequest &req, Sink &sink);

    void housekeeping();

private:
    void runManager(Process::Id parentId);
    void start();
    void stop();
    void worker(std::size_t id, Process::Id parentId, Worker::pointer worker);

    void cleanup(bool join);

    inline bool running() const { return *running_; }
    inline void running(bool val) { *running_ = val; }

    inline bi::interprocess_mutex& mutex() { return *mutex_; }
    inline bi::interprocess_condition& cond() { return *cond_; }

    Options options_;

    bi::mapped_region mem_;
    ManagedBuffer mb_;

    std::atomic<bool> *running_;
    ShRasterRequest::Deque *queue_;

    bi::interprocess_mutex *mutex_;
    bi::interprocess_condition *cond_;

    Process manager_;

    // TODO: rewrite to use processes
    Worker::map workers_;
};

GdalWarper::GdalWarper(const Options &options)
    : detail_(std::make_shared<Detail>(options))
{}

GdalWarper::Raster GdalWarper::warp(const RasterRequest &req, Sink &sink)
{
    return detail().warp(req, sink);
}

void GdalWarper::housekeeping()
{
    return detail().housekeeping();
}

GdalWarper::Detail::Detail(const Options &options)
    : options_(options)
    , mem_(bi::anonymous_shared_memory(std::size_t(1) << 30))
    , mb_(bi::create_only, mem_.get_address(), mem_.get_size())
    , running_(mb_.construct<std::atomic<bool>>(bi::anonymous_instance)(true))
    , queue_(mb_.construct<ShRasterRequest::Deque>
             (bi::anonymous_instance)
             (mb_.get_allocator<ShRasterRequest>()))
    , mutex_(mb_.construct<bi::interprocess_mutex>
             (bi::anonymous_instance)())
    , cond_(mb_.construct<bi::interprocess_condition>
            (bi::anonymous_instance)())
{
    start();
}

GdalWarper::Detail::~Detail()
{
    stop();
}

void GdalWarper::Detail::runManager(Process::Id parentId)
{
    dbglog::thread_id("gdal");
    LOG(info2) << "Started GDAL warper manager process.";

    auto isRunning([&]()
    {
        return running() && (parentId == ThisProcess::parentId());
    });

    auto thisId(ThisProcess::id());
    asio::io_service ios;
    asio::signal_set signals(ios, SIGCHLD);

    // handles signals
    std::function<void(const bs::error_code &e, int signo)> signalHandler;
    signalHandler = [&](const bs::error_code&, int signo)
    {
        signals.async_wait(signalHandler);
        if (signo != SIGCHLD) { return; }

        for (auto iworkers(workers_.begin()); iworkers != workers_.end(); ) {
            auto &worker(iworkers->second);
            try {
                // try to join this process
                auto id(worker->id());
                worker->join(true);
                LOG(warn2) << "Process " << id << " terminated unexpectedly.";
                worker->internalError(mutex());

                // process terminated -> remove
                iworkers = workers_.erase(iworkers);
            } catch (Process::Alive) {
                // process is still running, skip
                ++iworkers;
            }
        }
    };

    signals.async_wait(signalHandler);

    std::size_t idGenerator(0);

    // TODO: bind process with request so we can notify outer world that worker
    // process has died
    while (isRunning()) {
        if (workers_.size() < options_.processCount) {
            ios.notify_fork(asio::io_service::fork_prepare);
            auto id(++idGenerator);

            // create worker
            auto worker(Worker::create(mb_));
            // create process
            worker->attach(Process
                           (Process::Flags().quickExit(true)
                            , [this, id, thisId, worker, &ios]()
            {
                ios.notify_fork(asio::io_service::fork_child);
                this->worker(id, thisId, worker);
            }));

            // remember worker
            workers_.insert(Worker::map::value_type(worker->id(), worker));

            // notify fork and poll
            ios.notify_fork(asio::io_service::fork_parent);
            ios.poll();
            continue;
        }

        // poll and sleep a bit
        ios.poll();
        ::usleep(100000);
    }

    LOG(info3) << "Stopping GDAL warper worker processes.";
    cleanup(true);
    LOG(info3) << "Stopped GDAL warper manager process.";
}

void GdalWarper::Detail::start()
{
    manager_ = Process(Process::Flags().quickExit(true)
                       , &Detail::runManager, this, ThisProcess::id());
}

void GdalWarper::Detail::cleanup(bool join)
{
    // make not-running
    {
        Lock lock(mutex());
        running(false);
        cond().notify_all();
    }

    // cleanup
    while (!workers_.empty()) {
        auto head(workers_.begin());
        auto &worker(head->second);

        if (join) {
            // join worker
            worker->join();
        } else {
            // cock the gun and shoot
            worker->kill();
        }

        // notify any unfinished work
        worker->internalError(mutex());

        // get rid of this worker
        workers_.erase(head);
    }
}

void GdalWarper::Detail::housekeeping()
{
    try {
        manager_.join(true);
        LOG(warn3) << "Manager process terminated. Bailing out.";
        cleanup(false);
        throw AbandonAll("GDAL warper is screwed up pretty bad.");
    } catch (Process::Alive) {}
}

void GdalWarper::Detail::stop()
{
    LOG(info2) << "Stopping GDAL support.";
    {
        Lock lock(mutex());
        running(false);
        cond().notify_all();
    }

    LOG(info2) << "Waiting for processes to terminate.";

    // join manager process
    if (manager_.joinable()) {
        manager_.join();
    }
}

typedef GdalWarper::RasterRequest RasterRequest;

void GdalWarper::Detail::worker(std::size_t id, Process::Id parentId
                                , Worker::pointer worker)
{
    dbglog::thread_id(str(boost::format("gdal:%u") % id));
    LOG(info2) << "Spawned GDAL worker id:" << id << ".";
    DatasetCache cache;

    geo::Gdal::setOption("GDAL_ERROR_ON_LIBJPEG_WARNING", true);
    if (!options_.tmpRoot.empty()) {
        geo::Gdal::setOption("GDAL_DEFAULT_WMS_CACHE_PATH"
                             , (options_.tmpRoot / "gdalwmscache").string());
    }

    auto isRunning([&]()
    {
        return running() && (parentId == ThisProcess::parentId());
    });

    while (isRunning()) {
        try {
            ShRasterRequest::pointer req;

            {
                Lock lock(mutex());
                cond().timed_wait(lock, absTime(milliseconds(500)), [&]()
                {
                    return (!isRunning() || !queue_->empty());
                });
                if (!isRunning()) { break; }

                // empty queue -> nothing to do
                if (queue_->empty()) { continue; }

                // grab request
                req = queue_->back();
                queue_->pop_back();

                // associate request to this worker
                worker->associate(req);
            }

            try {
                req->set(mutex(), ::warp(cache, mb_, *req));
            } catch (const GenerateError &e) {
                req->setError(mutex(), e);
            } catch (const std::exception &e) {
                req->setError(mutex(), e);
            } catch (...) {
                req->setError(mutex(), "Unknown error.");
            }

            {
                // disassociate request from this worker
                Lock lock(mutex());
                worker->disassociate();
            }
        } catch (const std::exception &e) {
            LOG(err3)
                << "Uncaught exception in worker: <" << e.what()
                << ">. Going on.";
        }
    }

    LOG(info2) << "Terminated GDAL worker id:" << id << ".";
}

GdalWarper::Raster GdalWarper::Detail::warp(const RasterRequest &req
                                            , Sink &sink)
{
    Lock lock(mutex());
    ShRasterRequest::pointer shReq(ShRasterRequest::create(req, mb_));
    queue_->push_back(shReq);
    cond().notify_one();

    {
        // set aborter for this request
        ShRasterRequest::wpointer wreq(shReq);
        sink.setAborter([wreq, this]()
        {
            if (auto r = wreq.lock()) {
                r->setError
                    (mutex(), RequestAborted("Request has been aborted"));
            }
        });
    }

    return shReq->get(lock);
}
