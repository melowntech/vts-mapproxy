/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
#include "utility/eventcounter.hpp"

#include "geo/gdal.hpp"

#include "../error.hpp"
#include "../gdalsupport.hpp"
#include "process.hpp"
#include "datasetcache.hpp"
#include "types.hpp"
#include "operations.hpp"
#include "requests.hpp"
#include "workrequest.hpp"

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

/** TODO: check for allocation failures.
 */
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
        , work_()
        , done_(false)
        , error_(sm.get_allocator<char>())
        , errorType_(ErrorType::none)
        , ec_()
    {}

    ShRequest(const std::string &vectorDs
              , const DemDataset::list &rasterDs
              , const geo::heightcoding::Config &config
              , const boost::optional<std::string> &vectorGeoidGrid
              , const GdalWarper::OpenOptions &openOptions
              , const LayerEnhancer::map &layerEnhancers
              , ManagedBuffer &sm)
        : sm_(sm)
        , raster_()
        , heightcode_(sm.construct<ShHeightCode>
                      (bi::anonymous_instance)
                      (vectorDs, rasterDs, config, vectorGeoidGrid
                       , openOptions, layerEnhancers, sm, this))
        , work_()
        , done_(false)
        , error_(sm.get_allocator<char>())
        , errorType_(ErrorType::none)
        , ec_()
    {}

    ShRequest(const GdalWarper::WorkGenerator &workGenerator
              , ManagedBuffer &sm)
        : sm_(sm)
        , raster_()
        , heightcode_()
        , work_()
        , done_(false)
        , error_(sm.get_allocator<char>())
        , errorType_(ErrorType::none)
        , ec_()
    {
        work_ = workGenerator(sm);
    }

    ~ShRequest() {
        if (raster_) { sm_.destroy_ptr(raster_); }
        if (heightcode_) { sm_.destroy_ptr(heightcode_); }
        if (work_) { work_->destroy(); }
    }

    template <typename T>
    void setError(bi::interprocess_mutex &mutex, const T &what);

    void setError(Lock&, const char *message);
    void setError(Lock&, const std::exception &e);
    void setError(Lock&, const utility::HttpError &exc);
    void setError(Lock&, const EmptyImage &exc);
    void setError(Lock&, const FullImage &exc);
    void setError(Lock&, const EmptyGeoData &exc);

    GdalWarper::Raster getRaster(Lock &lock);
    GdalWarper::Raster getRaster(bi::interprocess_mutex &mutex);

    GdalWarper::Heightcoded::pointer getHeightcoded(Lock &lock);
    GdalWarper::Heightcoded::pointer getHeightcoded(bi::interprocess_mutex &mutex);

    WorkRequest::Response consumeWork(Lock &lock);

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
                          , const DemDataset::list &rasterDs
                          , const geo::heightcoding::Config &config
                          , const boost::optional<std::string> &vectorGeoidGrid
                          , const std::vector<std::string> &openOptions
                          , const LayerEnhancer::map &layerEnhancers
                          , ManagedBuffer &mb)
    {
        return pointer(mb.construct<ShRequest>
                       (bi::anonymous_instance)
                       (vectorDs, rasterDs, config, vectorGeoidGrid
                        , openOptions, layerEnhancers, mb)
                       , mb.get_allocator<void>()
                       , mb.get_deleter<ShRequest>());
    }

    static pointer create(const GdalWarper::WorkGenerator &workGenerator
                          , ManagedBuffer &mb)
    {
        return pointer(mb.construct<ShRequest>
                       (bi::anonymous_instance)(workGenerator, mb)
                       , mb.get_allocator<void>()
                       , mb.get_deleter<ShRequest>());
    }

private:
    ManagedBuffer &sm_;

    ShRaster *raster_;
    ShHeightCode *heightcode_;
    WorkRequest *work_;

    // response condition and flag
    bi::interprocess_condition cond_;
    bool done_;

    // response error
    String error_;

    enum class ErrorType {
        none, errorCode, emptyImage, fullImage
        , emptyGeoData
    };
    ErrorType errorType_;
    std::error_code ec_;
};

void ShRequest::process(bi::interprocess_mutex &mutex, DatasetCache &cache)
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
                                 , heightcode_->vectorGeoidGrid()
                                 , heightcode_->openOptions()
                                 , heightcode_->layerEnhancers()));
        return;
    }

    if (work_) {
        work_->process(mutex, cache);
        {
            Lock lock(mutex);
            done();
        }
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

void ShRequest::setError(Lock&, const FullImage &exc)
{
    if (!error_.empty()) { return; }
    error_.assign(exc.what());
    errorType_ = ErrorType::fullImage;
    done_ = true;
    cond_.notify_one();
}

void ShRequest::setError(Lock&, const EmptyGeoData &exc)
{
    if (!error_.empty()) { return; }
    error_.assign(exc.what());
    errorType_ = ErrorType::emptyGeoData;
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
        auto &sm(sm_);
        return GdalWarper::Raster(response, [&sm](cv::Mat *mat)
        {
            // deallocate data
            sm.deallocate(mat);
        });
    }

    switch (errorType_) {
    case ErrorType::none: break; // handled at the end of function

    case ErrorType::emptyImage: throw EmptyImage(asString(error_));

    case ErrorType::fullImage: throw FullImage(asString(error_));

    case ErrorType::emptyGeoData: throw EmptyGeoData(asString(error_));

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

GdalWarper::Heightcoded::pointer ShRequest::getHeightcoded(Lock &lock)
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

    if (auto *response = (heightcode_->response())) {
        auto &sm(sm_);
        return GdalWarper::Heightcoded::pointer
            (response, [&sm](GdalWarper::Heightcoded *block)
        {
            // deallocate data
            sm.deallocate(block);
        });
    }

    switch (errorType_) {
    case ErrorType::none: break; // handled at the end of function

    case ErrorType::emptyImage: throw EmptyImage(asString(error_));

    case ErrorType::fullImage: throw FullImage(asString(error_));

    case ErrorType::emptyGeoData: throw EmptyGeoData(asString(error_));

    case ErrorType::errorCode:
        utility::throwErrorCode(ec_, asString(error_));
    }

    throw std::runtime_error("Unknown exception!");
}

GdalWarper::Heightcoded::pointer
ShRequest::getHeightcoded(bi::interprocess_mutex &mutex)
{
    Lock lock(mutex);
    return getHeightcoded(lock);
}

WorkRequest::Response ShRequest::consumeWork(Lock &lock)
{
    cond_.wait(lock, [&]()
    {
        return done_;
    });

    if (!work_) {
        throw std::logic_error("This shared request is not handling a "
                               "job!");
    }

    if (!error_.empty()) {
        switch (errorType_) {
        case ErrorType::none: break; // handled at the end of function
        case ErrorType::emptyImage: throw EmptyImage(asString(error_));
        case ErrorType::fullImage: throw FullImage(asString(error_));
        case ErrorType::emptyGeoData: throw EmptyGeoData(asString(error_));
        case ErrorType::errorCode:
            utility::throwErrorCode(ec_, asString(error_));
        }
        throw std::runtime_error("Unknown exception!");
    }

    return work_->response(lock);
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

    Heightcoded::pointer
    heightcode(const std::string &vectorDs
               , const DemDataset::list &rasterDs
               , const geo::heightcoding::Config &config
               , const boost::optional<std::string> &vectorGeoidGrid
               , const GdalWarper::OpenOptions &openOptions
               , const LayerEnhancer::map &layerEnhancers
               , Aborter &aborter);

    WorkRequest::Response job(const WorkGenerator &workGenerator
                              , Aborter &aborter);

    void housekeeping();

    void stat(std::ostream &os) const;

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

    void reportShm();

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

    utility::EventCounter warpCounter_;
    utility::EventCounter heightcodeCounter_;
    utility::EventCounter shmCounter_;
    utility::EventCounter queueCounter_;
};

GdalWarper::GdalWarper(const Options &options, utility::Runnable &runnable)
    : detail_(std::make_shared<Detail>(options, runnable))
{}

GdalWarper::Raster GdalWarper::warp(const RasterRequest &req, Aborter &aborter)
{
    return detail().warp(req, aborter);
}

GdalWarper::Heightcoded::pointer
GdalWarper::heightcode(const std::string &vectorDs
                       , const DemDataset::list &rasterDs
                       , const geo::heightcoding::Config &config
                       , const boost::optional<std::string> &vectorGeoidGrid
                       , const GdalWarper::OpenOptions &openOptions
                       , const LayerEnhancer::map &layerEnhancers
                       , Aborter &aborter)
{
    return detail().heightcode(vectorDs, rasterDs, config, vectorGeoidGrid
                               , openOptions, layerEnhancers, aborter);
}

WorkRequest::Response GdalWarper::job(const WorkGenerator &workGenerator
                                      , Aborter &aborter)
{
    return detail().job(workGenerator, aborter);
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
    , warpCounter_(512)
    , heightcodeCounter_(512)
    , shmCounter_(512)
    , queueCounter_(512)
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
    // stop if there are no workers (yet)
    if (workers_.empty()) { return; }

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

        try {
            auto fworkers(workers_.find(u.pid));
            if (fworkers != workers_.end()) {
                fworkers->second->terminate();
            } else {
                // should not happen
                Process::kill(u.pid);
            }
        } catch (const std::exception &e) {
            LOG(warn2) << "Unable to kill worker process: <"
                       << e.what() << ">; ignoring.";
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

    // TODO: kill unresponsive processes

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
        // measure memory
        Lock lock(mutex());
        reportShm();
    } catch (...) {}

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
            } catch (const FullImage &e) {
                req->setError(mutex(), e);
            } catch (const EmptyGeoData &e) {
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

    LOG(info2) << "GDAL worker id:" << id << " finishing.";
}

void GdalWarper::Detail::reportShm()
{
    shmCounter_.eventMax(mb_.get_size() - mb_.get_free_memory());
    queueCounter_.eventMax(queue_->size());
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

    auto result(shReq->getRaster(lock));
    lock.unlock();

    warpCounter_.event();

    return result;
}

GdalWarper::Heightcoded::pointer GdalWarper::Detail
::heightcode(const std::string &vectorDs
             , const DemDataset::list &rasterDs
             , const geo::heightcoding::Config &config
             , const boost::optional<std::string> &vectorGeoidGrid
             , const GdalWarper::OpenOptions &openOptions
             , const LayerEnhancer::map &layerEnhancers
             , Aborter &aborter)
{
    Lock lock(mutex());
    ShRequest::pointer shReq
        (ShRequest::create(vectorDs, rasterDs, config, vectorGeoidGrid
                           , openOptions, layerEnhancers, mb_));
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

    auto result(shReq->getHeightcoded(lock));
    lock.unlock();

    heightcodeCounter_.event();

    return result;
}

WorkRequest::Response GdalWarper::Detail
::job(const WorkGenerator &workGenerator, Aborter &aborter)
{
    Lock lock(mutex());

    auto shReq(ShRequest::create(workGenerator, mb_));
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

    /** Consume response for a work request
     */
    return shReq->consumeWork(lock);
}

void GdalWarper::Detail::stat(std::ostream &os) const
{
    warpCounter_.averageAndMax(os, "gdal.warp.");
    heightcodeCounter_.averageAndMax(os, "gdal.heightcode.");
    shmCounter_.max(os, "gdal.shm.used.");
    os << "gdal.shm.total=" << mb_.get_size() << '\n';
    queueCounter_.max(os, "gdal.shm.enqueued.");
}

void GdalWarper::stat(std::ostream &os) const
{
    detail().stat(os);
}
