#include <sys/types.h>
#include <signal.h>

#include <new>
#include <array>
#include <atomic>
#include <thread>
#include <algorithm>

#include <boost/noncopyable.hpp>
#include <boost/format.hpp>
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "utility/errorcode.hpp"
#include "utility/procstat.hpp"

#include "geo/gdal.hpp"

#include "../error.hpp"
#include "../gdalsupport.hpp"
#include "./process.hpp"
#include "./datasetcache.hpp"
#include "./types.hpp"
#include "./operations.hpp"
#include "./requests.hpp"

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

class ShRequest : boost::noncopyable, public ShRequestBase {
public:
    typedef bi::deleter<ShRequest, SegmentManager> Deleter;
    typedef bi::shared_ptr<ShRequest, Allocator, Deleter> pointer;
    typedef bi::weak_ptr<ShRequest, Allocator, Deleter> wpointer;

    typedef bi::deque<pointer, bi::allocator<pointer, SegmentManager>> Deque;

    ShRequest(const GdalWarper::RasterRequest &other, ManagedBuffer &sm)
        : sm_(sm)
        , raster_(sm.construct<ShRaster>
                  (bi::anonymous_instance)(other, sm, this))
        , heightcode_()
        , navHeightcode_()
        , done_(false)
        , error_(sm.get_allocator<char>())
        , errorType_(ErrorType::none)
        , ec_()
    {}

    ShRequest(const std::string &vectorDs
              , const std::string &rasterDs
              , const geo::heightcoding::Config &config
              , const boost::optional<std::string> &geoidGrid
              , ManagedBuffer &sm)
        : sm_(sm)
        , raster_()
        , heightcode_(sm.construct<ShHeightCode>
                      (bi::anonymous_instance)
                      (vectorDs, rasterDs, config, geoidGrid, sm, this))
        , navHeightcode_()
        , done_(false)
        , error_(sm.get_allocator<char>())
        , errorType_(ErrorType::none)
        , ec_()
    {}

    ShRequest(const std::string &vectorDs
              , const GdalWarper::Navtile &navtile
              , const geo::heightcoding::Config &config
              , const boost::optional<std::string> &geoidGrid
              , ManagedBuffer &sm)
        : sm_(sm)
        , raster_()
        , heightcode_()
        , navHeightcode_(sm.construct<ShNavHeightCode>
                         (bi::anonymous_instance)
                         (vectorDs, navtile, config, geoidGrid, sm, this))
        , done_(false)
        , error_(sm.get_allocator<char>())
        , errorType_(ErrorType::none)
        , ec_()
    {}

    ~ShRequest() {
        if (raster_) { sm_.destroy_ptr(raster_); }
        if (heightcode_) { sm_.destroy_ptr(heightcode_); }
        if (navHeightcode_) { sm_.destroy_ptr(heightcode_); }
    }

    template <typename T>
    void setError(bi::interprocess_mutex &mutex, const T &what);

    void setError(Lock&, const char *message);
    void setError(Lock&, const std::exception &e);
    void setError(Lock&, const utility::HttpError &exc);
    void setError(Lock&, const EmptyImage &exc);

    GdalWarper::Raster getRaster(Lock &lock);
    GdalWarper::Raster getRaster(bi::interprocess_mutex &mutex);

    GdalWarper::Heighcoded::pointer getHeighcoded(Lock &lock);
    GdalWarper::Heighcoded::pointer getHeighcoded(bi::interprocess_mutex &mutex);

    virtual void done_impl();
    void process(bi::interprocess_mutex &mutex, DatasetCache &cache);

    static pointer create(const GdalWarper::RasterRequest &req
                          , ManagedBuffer &mb)
    {
        return pointer(mb.construct<ShRequest>
                       (bi::anonymous_instance)(req, mb)
                       , mb.get_allocator<void>()
                       , mb.get_deleter<ShRequest>());
    }

    static pointer create(const std::string &vectorDs
                          , const std::string &rasterDs
                          , const geo::heightcoding::Config &config
                          , const boost::optional<std::string> &geoidGrid
                          , ManagedBuffer &mb)
    {
        return pointer(mb.construct<ShRequest>
                       (bi::anonymous_instance)
                       (vectorDs, rasterDs, config, geoidGrid, mb)
                       , mb.get_allocator<void>()
                       , mb.get_deleter<ShRequest>());
    }

    static pointer create(const std::string &vectorDs
                          , const GdalWarper::Navtile &navtile
                          , const geo::heightcoding::Config &config
                          , const boost::optional<std::string> &geoidGrid
                          , ManagedBuffer &mb)
    {
        return pointer(mb.construct<ShRequest>
                       (bi::anonymous_instance)
                       (vectorDs, navtile, config, geoidGrid, mb)
                       , mb.get_allocator<void>()
                       , mb.get_deleter<ShRequest>());
    }

private:
    ManagedBuffer &sm_;

    ShRaster *raster_;
    ShHeightCode *heightcode_;
    ShNavHeightCode *navHeightcode_;

    // response condition and flag
    bi::interprocess_condition cond_;
    bool done_;

    // response error
    String error_;

    enum class ErrorType { none, errorCode, emptyImage };
    ErrorType errorType_;
    std::error_code ec_;
};

void ShRequest::process(bi::interprocess_mutex &mutex
                        , DatasetCache &cache)
{
    if (raster_) {
        raster_->response(mutex, ::warp(cache, sm_, *raster_));
        return;
    }

    if (heightcode_) {
        heightcode_->response
            (mutex, ::heightcode(cache, sm_
                                 , heightcode_->vectorDs()
                                 , heightcode_->rasterDs()
                                 , heightcode_->config()
                                 , heightcode_->geoidGrid()));
        return;
    }

    if (navHeightcode_) {
        navHeightcode_->response
            (mutex, ::heightcode(cache, sm_
                                 , navHeightcode_->vectorDs()
                                 , navHeightcode_->navtile()
                                 , navHeightcode_->config()
                                 , navHeightcode_->geoidGrid()));
        return;
    }

    setError(mutex, InternalError("No associated request."));
}

void ShRequest::done_impl()
{
    done_ = true;
    cond_.notify_one();
}

template <typename T>
void ShRequest::setError(bi::interprocess_mutex &mutex, const T &what)
{
    Lock lock(mutex);
    setError(lock, what);
}

void ShRequest::setError(Lock&, const char *message)
{
    if (!error_.empty()) { return; }
    error_.assign(message);
    errorType_ = ErrorType::errorCode;
    ec_ = make_error_code(utility::HttpCode::InternalServerError);
    done_ = true;
    cond_.notify_one();
}

void ShRequest::setError(Lock&, const utility::HttpError &exc)
{
    if (!error_.empty()) { return; }
    error_.assign(exc.what());
    errorType_ = ErrorType::errorCode;
    ec_ = exc.code();
    done_ = true;
    cond_.notify_one();
}

void ShRequest::setError(Lock&, const EmptyImage &exc)
{
    if (!error_.empty()) { return; }
    error_.assign(exc.what());
    errorType_ = ErrorType::emptyImage;
    done_ = true;
    cond_.notify_one();
}

void ShRequest::setError(Lock &lock, const std::exception &e)
{
    setError(lock, e.what());
}

GdalWarper::Raster ShRequest::getRaster(Lock &lock)
{
    cond_.wait(lock, [&]()
    {
        return done_;
    });

    if (!raster_) {
        throw std::logic_error("This shared request is not handling a "
                               "raster operation!");
    }

    if (auto *response = raster_->response()) {
        return GdalWarper::Raster(response, [&sm_](cv::Mat *mat)
        {
            // deallocate data
            sm_.deallocate(mat);
        });
    }

    switch (errorType_) {
    case ErrorType::none: break; // handled at the end of function

    case ErrorType::emptyImage: throw EmptyImage(asString(error_));

    case ErrorType::errorCode:
        utility::throwErrorCode(ec_, asString(error_));
    }

    throw std::runtime_error("Unknown exception!");
}

GdalWarper::Raster ShRequest::getRaster(bi::interprocess_mutex &mutex)
{
    Lock lock(mutex);
    return getRaster(lock);
}

GdalWarper::Heighcoded::pointer ShRequest::getHeighcoded(Lock &lock)
{
    cond_.wait(lock, [&]()
    {
        return done_;
    });

    // TODO: extend for other memblock-generating operations

    if (!(heightcode_ || navHeightcode_)) {
        throw std::logic_error("This shared request is not handling a "
                               "vector operation!");
    }

    if (auto *response = (heightcode_
                          ? heightcode_->response()
                          : navHeightcode_->response()))
    {
        return GdalWarper::Heighcoded::pointer
            (response, [&sm_](GdalWarper::Heighcoded *block)
        {
            // deallocate data
            sm_.deallocate(block);
        });
    }

    switch (errorType_) {
    case ErrorType::none: break; // handled at the end of function

    case ErrorType::emptyImage: throw EmptyImage(asString(error_));

    case ErrorType::errorCode:
        utility::throwErrorCode(ec_, asString(error_));
    }

    throw std::runtime_error("Unknown exception!");
}

GdalWarper::Heighcoded::pointer
ShRequest::getHeighcoded(bi::interprocess_mutex &mutex)
{
    Lock lock(mutex);
    return getHeighcoded(lock);
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

    void terminate() {
        // terminate the process, ignore all errors
        if (process_.joinable()) {
            try {
                process_.terminate();
            } catch(...) {}
        }
    }

    bool killed() const { return process_.killed(); }

    void associate(const ShRequest::pointer &req) { req_ = req; }

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
    ShRequest::pointer req_;
};

} // namespace

class GdalWarper::Detail
{
public:
    Detail(const Options &options, utility::Runnable &runnable);
    ~Detail();

    Raster warp(const RasterRequest &req, Aborter &aborter);

    Heighcoded::pointer
    heightcode(const std::string &vectorDs
               , const std::string &rasterDs
               , const geo::heightcoding::Config &config
               , const boost::optional<std::string> &geoidGrid
               , Aborter &aborter);

    Heighcoded::pointer
    heightcode(const std::string &vectorDs
               , const Navtile &navtile
               , const geo::heightcoding::Config &config
               , const boost::optional<std::string> &geoidGrid
               , Aborter &aborter);

    void housekeeping();

private:
    void runManager(Process::Id parentId);
    void start();
    void stop();
    void worker(std::size_t id, Process::Id parentId, Worker::pointer worker);

    void cleanup(bool join);

    void killLeviathan();

    inline bool running() const { return *running_; }
    inline void running(bool val) { *running_ = val; }

    inline bi::interprocess_mutex& mutex() { return *mutex_; }
    inline bi::interprocess_condition& cond() { return *cond_; }

    Options options_;
    utility::Runnable &runnable_;

    bi::mapped_region mem_;
    ManagedBuffer mb_;

    std::atomic<bool> *running_;
    ShRequest::Deque *queue_;

    bi::interprocess_mutex *mutex_;
    bi::interprocess_condition *cond_;

    Process manager_;

    Worker::map workers_;
};

GdalWarper::GdalWarper(const Options &options, utility::Runnable &runnable)
    : detail_(std::make_shared<Detail>(options, runnable))
{}

GdalWarper::Raster GdalWarper::warp(const RasterRequest &req, Aborter &aborter)
{
    return detail().warp(req, aborter);
}

GdalWarper::Heighcoded::pointer
GdalWarper::heightcode(const std::string &vectorDs
                       , const std::string &rasterDs
                       , const geo::heightcoding::Config &config
                       , const boost::optional<std::string> &geoidGrid
                       , Aborter &aborter)
{
    return detail().heightcode(vectorDs, rasterDs, config, geoidGrid, aborter);
}

GdalWarper::Heighcoded::pointer
GdalWarper::heightcode(const std::string &vectorDs
                       , const Navtile &navtile
                       , const geo::heightcoding::Config &config
                       , const boost::optional<std::string> &geoidGrid
                       , Aborter &aborter)
{
    return detail().heightcode(vectorDs, navtile, config, geoidGrid, aborter);
}

void GdalWarper::housekeeping()
{
    return detail().housekeeping();
}

GdalWarper::Detail::Detail(const Options &options
                           , utility::Runnable &runnable)
    : options_(options), runnable_(runnable)
    , mem_(bi::anonymous_shared_memory(std::size_t(1) << 30))
    , mb_(bi::create_only, mem_.get_address(), mem_.get_size())
    , running_(mb_.construct<std::atomic<bool>>(bi::anonymous_instance)(true))
    , queue_(mb_.construct<ShRequest::Deque>
             (bi::anonymous_instance)
             (mb_.get_allocator<ShRequest>()))
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
        return (running() && (parentId == ThisProcess::parentId())
                && runnable_.isRunning());
    });

    auto thisId(ThisProcess::id());
    asio::io_service ios;
    asio::signal_set signals(ios, SIGCHLD);
    asio::steady_timer killTimer(ios);

    // handles signals
    std::function<void(const bs::error_code &e, int signo)> signalHandler;
    std::function<void(const bs::error_code &e)> killTimeoutHandler;

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
                if (worker->killed()) {
                    LOG(info1)
                        << "Collected process " << id << ".";
                } else {
                    LOG(warn2)
                        << "Process " << id << " terminated unexpectedly.";
                }
                worker->internalError(mutex());

                // process terminated -> remove
                iworkers = workers_.erase(iworkers);
            } catch (Process::Alive) {
                // process is still running, skip
                ++iworkers;
            }
        }
    };

    killTimeoutHandler = [&](const bs::error_code&)
    {
        killTimer.expires_from_now
        (std::chrono::seconds(options_.rssCheckPeriod));
        killTimer.async_wait(killTimeoutHandler);
        killLeviathan();
    };

    // launch handler to let them register
    signalHandler({}, 0);
    killTimeoutHandler({});

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

void GdalWarper::Detail::killLeviathan()
{
    utility::PidList pids;
    for (const auto &item : workers_) { pids.push_back(item.first); }

    auto stats(utility::getProcStat(pids));

    struct Usage {
        Process::Id pid;
        std::size_t mem;

        Usage(const utility::ProcStat &ps)
            : pid(ps.pid), mem(ps.occupies())
        {
            // remove shared memory count; stop at zero
            if (mem > ps.shared) {
                mem -= ps.shared;
            } else {
                mem = 0;
            }
        }

        bool operator<(const Usage &u) const { return mem > u.mem; }

        typedef std::vector<Usage> list;
    };

    Usage::list usage;
    std::size_t total(0);
    for (const auto &ps : stats) {
        usage.emplace_back(ps);
        total += usage.back().mem;
    }
    std::sort(usage.begin(), usage.end());

    // compute limit in kilobytes
    const std::size_t limit(options_.rssLimit * 1024);

    LOG(info1) << "Total: " << total << " KB, limit: " << limit << " KB";

    // process whole process list sorted by memory usage
    for (const auto &u : usage) {
        if (total < limit) { return; }

        // a leviathan found! kill it with fire
        LOG(info3)
            << "Killing large GDAL process " << u.pid
            << " occupying " << (double(total) / 1024) << "MB of memory.";

        auto fworkers(workers_.find(u.pid));
        if (fworkers != workers_.end()) {
            fworkers->second->terminate();
        } else {
            // should not happen
            Process::kill(u.pid);
        }
        total -= u.mem;
    }
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
        return (running() && (parentId == ThisProcess::parentId())
                && runnable_.isRunning());
    });

    while (isRunning()) {
        try {
            ShRequest::pointer req;

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
                req->process(mutex(), cache);
            } catch (const utility::HttpError &e) {
                req->setError(mutex(), e);
            } catch (const EmptyImage &e) {
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

        if (cache.worn()) { break; }
    }

    LOG(info2) << "GDAL worker id" << id << " finishing.";
}

GdalWarper::Raster GdalWarper::Detail::warp(const RasterRequest &req
                                               , Aborter &aborter)
{
    Lock lock(mutex());
    ShRequest::pointer shReq(ShRequest::create(req, mb_));
    queue_->push_back(shReq);
    cond().notify_one();

    {
        // set aborter for this request
        ShRequest::wpointer wreq(shReq);
        aborter.setAborter([wreq, this]()
        {
            if (auto r = wreq.lock()) {
                r->setError
                    (mutex(), RequestAborted("Request has been aborted"));
            }
        });
    }

    return shReq->getRaster(lock);
}

GdalWarper::Heighcoded::pointer
GdalWarper::Detail::heightcode(const std::string &vectorDs
                               , const std::string &rasterDs
                               , const geo::heightcoding::Config &config
                               , const boost::optional<std::string> &geoidGrid
                               , Aborter &aborter)
{
    Lock lock(mutex());
    ShRequest::pointer shReq
        (ShRequest::create(vectorDs, rasterDs, config, geoidGrid, mb_));
    queue_->push_back(shReq);
    cond().notify_one();

    {
        // set aborter for this request
        ShRequest::wpointer wreq(shReq);
        aborter.setAborter([wreq, this]()
        {
            if (auto r = wreq.lock()) {
                r->setError
                    (mutex(), RequestAborted("Request has been aborted"));
            }
        });
    }

    return shReq->getHeighcoded(lock);
}

GdalWarper::Heighcoded::pointer
GdalWarper::Detail::heightcode(const std::string &vectorDs
                               , const Navtile &navtile
                               , const geo::heightcoding::Config &config
                               , const boost::optional<std::string> &geoidGrid
                               , Aborter &aborter)
{
    Lock lock(mutex());
    ShRequest::pointer shReq
        (ShRequest::create(vectorDs, navtile, config, geoidGrid, mb_));
    queue_->push_back(shReq);
    cond().notify_one();

    {
        // set aborter for this request
        ShRequest::wpointer wreq(shReq);
        aborter.setAborter([wreq, this]()
        {
            if (auto r = wreq.lock()) {
                r->setError
                    (mutex(), RequestAborted("Request has been aborted"));
            }
        });
    }

    return shReq->getHeighcoded(lock);
}
