#include <sys/types.h>

#include <new>
#include <array>
#include <atomic>
#include <thread>

#include <boost/format.hpp>

#include <boost/interprocess/smart_ptr/shared_ptr.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/smart_ptr/deleter.hpp>
#include <boost/interprocess/managed_external_buffer.hpp>
#include <boost/interprocess/indexes/flat_map_index.hpp>
#include <boost/interprocess/anonymous_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/containers/deque.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include "imgproc/rastermask/cvmat.hpp"

#include "../error.hpp"
#include "../gdalsupport.hpp"
#include "./process.hpp"

namespace bi = boost::interprocess;

namespace {
typedef bi::basic_managed_external_buffer<
    char
    , bi::rbtree_best_fit<bi::mutex_family, void*>
    , bi::flat_map_index> ManagedBuffer;
typedef ManagedBuffer::segment_manager SegmentManager;

typedef bi::allocator<void, SegmentManager> Allocator;
typedef bi::deleter<void, SegmentManager> Deleter;
typedef bi::shared_ptr<cv::Mat, Allocator, Deleter> MatPointer;
typedef bi::shared_ptr<std::string, Allocator, Deleter> StringPointer;

typedef bi::basic_string<
    char, std::char_traits<char>
    , bi::allocator<char, SegmentManager>
    > String;

typedef bi::scoped_lock<bi::interprocess_mutex> Lock;

class ShRasterRequest {
public:
    typedef bi::shared_ptr<ShRasterRequest, Allocator, Deleter> pointer;

    typedef bi::deque<pointer, bi::allocator<pointer, SegmentManager>> Deque;

    ShRasterRequest(const GdalWarper::RasterRequest &other
                    , ManagedBuffer &sm)
        : sm_(sm)
        , operation_(other.operation)
        , dataset_(other.dataset.data()
                 , other.dataset.size()
                 , sm.get_allocator<char>())
        , mask_(sm.get_allocator<char>())
        , srs_(other.srs.srs.data()
               , other.srs.srs.size()
               , sm.get_allocator<char>())
        , srsType_(other.srs.type)
        , extents_(other.extents)
        , size_(other.size)
        , resampling_(other.resampling)
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
             , asOptional(mask_)
             , geo::SrsDefinition(asString(srs_), srsType_)
             , extents_, size_, resampling_);
    }

    void setError(bi::interprocess_mutex &mutex, const char *message);
    void setError(bi::interprocess_mutex &mutex, const std::exception &e);
    void setError(bi::interprocess_mutex &mutex, const GenerateError &e);

    GdalWarper::Raster get(bi::interprocess_mutex &mutex);
    void set(bi::interprocess_mutex &mutex, cv::Mat *response);

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
    String mask_;
    String srs_;
    geo::SrsDefinition::Type srsType_;
    math::Extents2 extents_;
    math::Size2 size_;
    geo::GeoDataset::Resampling resampling_;

    // response condition and flag
    bi::interprocess_condition cond_;
    bool done_;

    // response matrix
    cv::Mat *response_;

    // response error
    String error_;
    GenerateError::RaiseError raiseError_;
};

void ShRasterRequest::setError(bi::interprocess_mutex &mutex
                               , const char *message)
{
    Lock lock(mutex);
    error_.assign(message);
    raiseError_ = &GenerateError::runtimeError;
    done_ = true;
}

void ShRasterRequest::setError(bi::interprocess_mutex &mutex
                               , const GenerateError &e)
{
    Lock lock(mutex);
    error_.assign(e.what());
    raiseError_ = e.getRaise();
    done_ = true;
}

void ShRasterRequest::setError(bi::interprocess_mutex &mutex
                               , const std::exception &e)
{
    setError(mutex, e.what());
}

void ShRasterRequest::set(bi::interprocess_mutex &mutex, cv::Mat *response)
{
    Lock lock(mutex);
    // TODO: check for multiple assignments
    response_ = response;
    done_ = true;
    cond_.notify_one();
}

GdalWarper::Raster ShRasterRequest::get(bi::interprocess_mutex &mutex)
{
    Lock lock(mutex);
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

} // namespace

class GdalWarper::Detail
{
public:
    Detail(unsigned int processCount);
    ~Detail();

    Raster warp(const RasterRequest &req);

private:
    void runManager();
    void start();
    void stop();
    void worker(std::size_t id);

    inline bool running() const { return *running_; }
    inline void running(bool val) { *running_ = val; }

    inline bi::interprocess_mutex& mutex() { return *mutex_; }
    inline bi::interprocess_condition& cond() { return *cond_; }

    Raster rasterFromSharedMat(cv::Mat *mat);

    cv::Mat* warpImpl(const RasterRequest &req);
    cv::Mat* warpImage(const std::string &dataset
                       , const boost::optional<std::string> &maskDataset
                       , const geo::SrsDefinition &srs
                       , const math::Extents2 &extents
                       , const math::Size2 &size
                       , geo::GeoDataset::Resampling resampling);

    cv::Mat* warpMask(const std::string &dataset
                      , const boost::optional<std::string> &maskDataset
                      , const geo::SrsDefinition &srs
                      , const math::Extents2 &extents
                      , const math::Size2 &size
                      , geo::GeoDataset::Resampling resampling);

    unsigned int processCount_;

    bi::mapped_region mem_;
    ManagedBuffer mb_;

    std::atomic<bool> *running_;
    ShRasterRequest::Deque *queue_;

    bi::interprocess_mutex *mutex_;
    bi::interprocess_condition *cond_;

    Process manager_;

    // TODO: rewrite to use processes
    std::vector<Process> workers_;
};

GdalWarper::GdalWarper(unsigned int processCount)
    : detail_(std::make_shared<Detail>(processCount))
{}

GdalWarper::Raster GdalWarper::warp(const RasterRequest &req)
{
    return detail().warp(req);
}

namespace {

cv::Mat* allocateMat(ManagedBuffer &mb
                     , const math::Size2 &size, int type)
{
    // calculate sizes
    const auto dataSize(math::area(size) * CV_ELEM_SIZE(type));
    const auto matSize(sizeof(cv::Mat) + dataSize);

    // create raw memory to hold matrix and data
    char *raw(static_cast<char*>(mb.allocate(matSize)));

    // allocate matrix in raw data block
    return new (raw) cv::Mat(size.height, size.width, type
                             , raw + sizeof(cv::Mat));
}

} // namespace

GdalWarper::Detail::Detail(unsigned int processCount)
    : processCount_(processCount)
    , mem_(bi::anonymous_shared_memory(1 << 22))
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

void GdalWarper::Detail::runManager()
{
    LOG(info3) << "Started GDAL warper manager process.";
    while (running()) {
        if (workers_.size() < processCount_) {
            workers_.emplace_back(Process::Flags().quickExit(true)
                                  , &Detail::worker, this, workers_.size());
            continue;
        }

        sleep(1);
    }

    LOG(info3) << "Stopping GDAL warper worker processes.";

    while (!workers_.empty()) {
        workers_.back().join();
        workers_.pop_back();
    }

    LOG(info3) << "Stopped GDAL warper manager process.";
}

void GdalWarper::Detail::start()
{
    manager_ = Process(Process::Flags().quickExit(true)
                       , &Detail::runManager, this);
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

    manager_.join();
}

void GdalWarper::Detail::worker(std::size_t id)
{
    dbglog::thread_id(str(boost::format("gdal:%u") % id));
    LOG(info2) << "Spawned GDAL worker id:" << id << ".";

    while (running()) {
        try {
            ShRasterRequest::pointer req;

            {
                Lock lock(mutex());
                cond().wait(lock, [&]()
                {
                    return (!running() || !queue_->empty());
                });
                if (!running()) { break; }

                if (!queue_->empty()) {
                    req = queue_->back();
                    queue_->pop_back();
                }
            }

            if (req) {
                try {
                    req->set(mutex(), warpImpl(*req));
                } catch (const GenerateError &e) {
                    req->setError(mutex(), e);
                } catch (const std::exception &e) {
                    req->setError(mutex(), e);
                } catch (...) {
                    req->setError(mutex(), "Unknown error.");
                }
            }
        } catch (const std::exception &e) {
            LOG(err3)
                << "Uncaught exception in worker: <" << e.what()
                << ">. Going on.";
        }
    }

    LOG(info2) << "Terminated GDAL worker id:" << id << ".";
}

GdalWarper::Raster GdalWarper::Detail::rasterFromSharedMat(cv::Mat *mat)
{
    return GdalWarper::Raster(mat, [this](cv::Mat *mat)
    {
        // deallocate data
        mb_.deallocate(mat);
    });
}

GdalWarper::Raster GdalWarper::Detail::warp(const RasterRequest &req)
{
    ShRasterRequest::pointer shReq;
    {
        Lock lock(mutex());
        shReq = ShRasterRequest::pointer(mb_.construct<ShRasterRequest>
                                         (bi::anonymous_instance)
                                         (req, mb_)
                                         , mb_.get_allocator<void>()
                                         , mb_.get_deleter<void>());

        queue_->push_back(shReq);
        cond().notify_one();
    }

    return shReq->get(mutex());
}

cv::Mat* GdalWarper::Detail::warpImpl(const RasterRequest &req)
{
    switch (req.operation) {
    case RasterRequest::Operation::image:
        return warpImage
            (req.dataset, req.mask, req.srs, req.extents, req.size
             , req.resampling);
    case RasterRequest::Operation::mask:
        return warpMask
            (req.dataset, req.mask, req.srs, req.extents, req.size
             , req.resampling);
    }
    throw;
}

cv::Mat*
GdalWarper::Detail::warpImage(const std::string &dataset
                              , const boost::optional<std::string> &maskDataset
                              , const geo::SrsDefinition &srs
                              , const math::Extents2 &extents
                              , const math::Size2 &size
                              , geo::GeoDataset::Resampling resampling)
{
    auto src(geo::GeoDataset::open(dataset));
    auto dst(geo::GeoDataset::deriveInMemory(src, srs, size, extents));
    src.warpInto(dst, resampling);

    if (dst.cmask().empty()) {
        throw NotFound("No valid data.");
    }

    // apply mask set if defined
    if (maskDataset) {
        auto srcMask(geo::GeoDataset::open(*maskDataset));
        auto dstMask(geo::GeoDataset::deriveInMemory
                     (srcMask, srs, size, extents));
        srcMask.warpInto(dstMask, resampling);
        dst.applyMask(dstMask.cmask());
    }

    // grab destination
    auto dstMat(dst.cdata());
    auto type(CV_MAKETYPE(CV_8U, dstMat.channels()));

    auto *tile(allocateMat(mb_, size, type));
    dstMat.convertTo(*tile, type);
    return tile;
}

cv::Mat*
GdalWarper::Detail::warpMask(const std::string &dataset
                             , const boost::optional<std::string> &maskDataset
                             , const geo::SrsDefinition &srs
                             , const math::Extents2 &extents
                             , const math::Size2 &size
                             , geo::GeoDataset::Resampling resampling)
{
    // TODO: execute in external process

    auto src(geo::GeoDataset::open(dataset));
    auto dst(geo::GeoDataset::deriveInMemory(src, srs, size, extents));
    src.warpInto(dst, resampling);

    if (dst.cmask().empty()) {
        throw NotFound("No valid data.");
    }

    // apply mask set if defined
    if (maskDataset) {
        auto srcMask(geo::GeoDataset::open(*maskDataset));
        auto dstMask(geo::GeoDataset::deriveInMemory
                     (srcMask, srs, size, extents));
        srcMask.warpInto(dstMask, resampling);
        dst.applyMask(dstMask.cmask());
    }

    auto &cmask(dst.cmask());
    auto *mask(allocateMat(mb_, maskMatSize(cmask), maskMatDataType(cmask)));
    asCvMat(*mask, cmask);
    return mask;
}
