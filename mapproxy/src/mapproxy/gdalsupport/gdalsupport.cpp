#include <sys/types.h>

#include <new>
#include <array>
#include <atomic>
#include <thread>

#include <boost/noncopyable.hpp>
#include <boost/format.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "utility/errorcode.hpp"

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

std::string asString(const String &str) {
    return { str.data(), str.size() };
}

boost::optional<std::string> asOptional(const String &str) {
    if (str.empty()) { return {}; }
    return std::string(str.data(), str.size());
}

boost::optional<geo::SrsDefinition>
asOptional(const String &str, geo::SrsDefinition::Type type)
{
    if (str.empty()) { return {}; }
    return geo::SrsDefinition(std::string(str.data(), str.size()), type);
}

typedef boost::posix_time::milliseconds milliseconds;

class ShRequest;

class ShRaster : boost::noncopyable {
public:
    ShRaster(const GdalWarper::RasterRequest &other
                    , ManagedBuffer &sm, ShRequest *owner)
        : sm_(sm), owner_(owner)
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
        , response_()
    {
        if (other.mask) {
            mask_.assign(other.mask->data(), other.mask->size());
        }
    }

    ~ShRaster() {
        if (response_) { sm_.deallocate(response_); }
    }

    operator GdalWarper::RasterRequest() const {
        return GdalWarper::RasterRequest
            (operation_
             , std::string(dataset_.data(), dataset_.size())
             , geo::SrsDefinition(asString(srs_), srsType_)
             , extents_, size_, resampling_
             , asOptional(mask_));
    }

    /** Steals response.
     */
    cv::Mat* response() {
        auto response(response_);
        response_ = 0;
        return response;
    }

    void response(bi::interprocess_mutex &mutex, cv::Mat *response);

private:
    ManagedBuffer &sm_;
    ShRequest *owner_;

    GdalWarper::RasterRequest::Operation operation_;
    String dataset_;
    String srs_;
    geo::SrsDefinition::Type srsType_;
    math::Extents2 extents_;
    math::Size2 size_;
    geo::GeoDataset::Resampling resampling_;
    String mask_;

    // response matrix
    cv::Mat *response_;
};

class ShHeightCode : boost::noncopyable {
public:
    ShHeightCode(const std::string &vectorDs, const std::string &rasterDs
                 , const geo::HeightCodingConfig &config
                 , ManagedBuffer &sm, ShRequest *owner)
        : sm_(sm), owner_(owner)
        , vectorDs_(vectorDs.data(), vectorDs.size()
                    , sm.get_allocator<char>())
        , rasterDs_(rasterDs.data(), rasterDs.size()
                    , sm.get_allocator<char>())

        , workingSrs_(sm.get_allocator<char>())
        , workingSrsType_()

        , outputSrs_(sm.get_allocator<char>())
        , outputSrsType_()

        , clipExtents_(config.clipExtents)
        , format_(config.format)

        , response_()
    {

        if (config.workingSrs) {
            workingSrs_.assign(config.workingSrs->srs.data()
                               , config.workingSrs->srs.size());
            workingSrsType_ = config.workingSrs->type;
        }

        if (config.outputSrs) {
            outputSrs_.assign(config.outputSrs->srs.data()
                              , config.outputSrs->srs.size());
            outputSrsType_ = config.outputSrs->type;
        }

        if (config.layers) {
            layers_ = boost::in_place(sm.get_allocator<String>());

            for (const auto &str : *config.layers) {
                layers_->push_back(String(str.data(), str.size()
                                          , sm.get_allocator<char>()));
            }
        }
    }

    ~ShHeightCode() {
        if (response_) { sm_.deallocate(response_); }
    }

    std::string vectorDs() const { return asString(vectorDs_); }

    std::string rasterDs() const { return asString(rasterDs_); }

    geo::HeightCodingConfig config() const {
        geo::HeightCodingConfig config;
        config.workingSrs = asOptional(workingSrs_, workingSrsType_);
        config.outputSrs = asOptional(outputSrs_, outputSrsType_);
        config.clipExtents = clipExtents_;

        if (layers_) {
            config.layers = boost::in_place();
            for (const auto &str : *layers_) {
                config.layers->emplace_back(str.data(), str.size());
            }
        }

        config.format = format_;

        return config;
    }


    /** Steals response.
     */
    GdalWarper::MemBlock* response() {
        auto response(response_);
        response_ = 0;
        return response;
    }

    void response(bi::interprocess_mutex &mutex
                  , GdalWarper::MemBlock *response);

private:
    ManagedBuffer &sm_;
    ShRequest *owner_;

    String vectorDs_;
    String rasterDs_;

    String workingSrs_;
    geo::SrsDefinition::Type workingSrsType_;

    String outputSrs_;
    geo::SrsDefinition::Type outputSrsType_;

    boost::optional<StringVector> layers_;

    boost::optional<math::Extents2> clipExtents_;

    geo::VectorFormat format_;

    // response memory block
    GdalWarper::MemBlock *response_;
};

class ShRequest : boost::noncopyable {
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
        , done_(false)
        , error_(sm.get_allocator<char>())
        , errorType_(ErrorType::none)
        , ec_()
    {}

    ShRequest(const std::string &vectorDs
              , const std::string &rasterDs
              , const geo::HeightCodingConfig &config
              , ManagedBuffer &sm)
        : sm_(sm)
        , raster_()
        , heightcode_(sm.construct<ShHeightCode>
                      (bi::anonymous_instance)
                      (vectorDs, rasterDs, config, sm, this))
        , done_(false)
        , error_(sm.get_allocator<char>())
        , errorType_(ErrorType::none)
        , ec_()
    {}

    ~ShRequest() {
        if (raster_) { sm_.destroy_ptr(raster_); }
    }

    template <typename T>
    void setError(bi::interprocess_mutex &mutex, const T &what);

    void setError(Lock&, const char *message);
    void setError(Lock&, const std::exception &e);
    void setError(Lock&, const utility::HttpError &exc);
    void setError(Lock&, const EmptyImage &exc);

    GdalWarper::Raster getRaster(Lock &lock);
    GdalWarper::Raster getRaster(bi::interprocess_mutex &mutex);

    GdalWarper::MemBlock::pointer getMemBlock(Lock &lock);
    GdalWarper::MemBlock::pointer getMemBlock(bi::interprocess_mutex &mutex);

    void done();
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
                          , const geo::HeightCodingConfig &config
                          , ManagedBuffer &mb)
    {
        return pointer(mb.construct<ShRequest>
                       (bi::anonymous_instance)(vectorDs, rasterDs, config, mb)
                       , mb.get_allocator<void>()
                       , mb.get_deleter<ShRequest>());
    }

private:
    ManagedBuffer &sm_;

    ShRaster *raster_;
    ShHeightCode *heightcode_;

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
                                 , heightcode_->config()));
        return;
    }

    setError(mutex, InternalError("No associated request."));
}

void ShRequest::done()
{

    done_ = true;
    cond_.notify_one();
}

void ShRaster::response(bi::interprocess_mutex &mutex, cv::Mat *response)
{
    Lock lock(mutex);
    if (response_) { return; }
    response_ = response;
    owner_->done();
}

void ShHeightCode::response(bi::interprocess_mutex &mutex
                            , GdalWarper::MemBlock *response)
{
    Lock lock(mutex);
    if (response_) { return; }
    response_ = response;
    owner_->done();
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

GdalWarper::MemBlock::pointer ShRequest::getMemBlock(Lock &lock)
{
    cond_.wait(lock, [&]()
    {
        return done_;
    });

    // TODO: extend for other memblock-generating operations

    if (!heightcode_) {
        throw std::logic_error("This shared request is not handling a "
                               "vector operation!");
    }

    if (auto *response = heightcode_->response()) {
        return GdalWarper::MemBlock::pointer
            (response, [&sm_](GdalWarper::MemBlock *block)
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

GdalWarper::MemBlock::pointer
ShRequest::getMemBlock(bi::interprocess_mutex &mutex)
{
    Lock lock(mutex);
    return getMemBlock(lock);
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
    Detail(const Options &options);
    ~Detail();

    Raster warp(const RasterRequest &req, Sink &sink);

    MemBlock::pointer heightcode(const std::string &vectorDs
                                 , const std::string &rasterDs
                                 , const geo::HeightCodingConfig &config
                                 , Sink &sink);

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
    ShRequest::Deque *queue_;

    bi::interprocess_mutex *mutex_;
    bi::interprocess_condition *cond_;

    Process manager_;

    Worker::map workers_;
};

GdalWarper::GdalWarper(const Options &options)
    : detail_(std::make_shared<Detail>(options))
{}

GdalWarper::Raster GdalWarper::warp(const RasterRequest &req, Sink &sink)
{
    return detail().warp(req, sink);
}

GdalWarper::MemBlock::pointer
GdalWarper::heightcode(const std::string &vectorDs
                       , const std::string &rasterDs
                       , const geo::HeightCodingConfig &config
                       , Sink &sink)
{
    return detail().heightcode(vectorDs, rasterDs, config, sink);
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
    }

    LOG(info2) << "Terminated GDAL worker id:" << id << ".";
}

GdalWarper::Raster GdalWarper::Detail::warp(const RasterRequest &req
                                               , Sink &sink)
{
    Lock lock(mutex());
    ShRequest::pointer shReq(ShRequest::create(req, mb_));
    queue_->push_back(shReq);
    cond().notify_one();

    {
        // set aborter for this request
        ShRequest::wpointer wreq(shReq);
        sink.setAborter([wreq, this]()
        {
            if (auto r = wreq.lock()) {
                r->setError
                    (mutex(), RequestAborted("Request has been aborted"));
            }
        });
    }

    return shReq->getRaster(lock);
}

GdalWarper::MemBlock::pointer
GdalWarper::Detail::heightcode(const std::string &vectorDs
                               , const std::string &rasterDs
                               , const geo::HeightCodingConfig &config
                               , Sink &sink)
{
    Lock lock(mutex());
    ShRequest::pointer shReq
        (ShRequest::create(vectorDs, rasterDs, config, mb_));
    queue_->push_back(shReq);
    cond().notify_one();

    {
        // set aborter for this request
        ShRequest::wpointer wreq(shReq);
        sink.setAborter([wreq, this]()
        {
            if (auto r = wreq.lock()) {
                r->setError
                    (mutex(), RequestAborted("Request has been aborted"));
            }
        });
    }

    return shReq->getMemBlock(lock);
}
