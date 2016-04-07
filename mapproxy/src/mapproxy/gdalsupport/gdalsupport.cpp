#include <boost/interprocess/smart_ptr/shared_ptr.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/smart_ptr/deleter.hpp>
#include <boost/interprocess/managed_external_buffer.hpp>
#include <boost/interprocess/indexes/flat_map_index.hpp>
#include <boost/interprocess/anonymous_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include "imgproc/rastermask/cvmat.hpp"

#include "../error.hpp"
#include "../gdalsupport.hpp"

namespace bi = boost::interprocess;

namespace {
    typedef bi::basic_managed_external_buffer<
        char
        , bi::rbtree_best_fit<bi::mutex_family, void*>
        , bi::flat_map_index> ManagedBuffer;
    typedef ManagedBuffer::segment_manager SegmentManager;
    typedef bi::allocator<void, SegmentManager> Allocator;
    typedef bi::deleter<char, SegmentManager> Deleter;

    typedef bi::shared_ptr<char, Allocator, Deleter> CharPointer;
} // namespace

class GdalWarper::Detail
{
public:
    Detail(unsigned int processCount)
        : processCount_(processCount)
        , mem_(bi::anonymous_shared_memory(1 << 22))
        , mb_(bi::create_only, mem_.get_address(), mem_.get_size())
    {
    }

    Raster warpImage(const std::string &dataset
                     , const boost::optional<std::string> &maskDataset
                     , const geo::SrsDefinition &srs
                     , const math::Extents2 &extents
                     , const math::Size2 &size
                     , geo::GeoDataset::Resampling resampling);

    Raster warpMask(const std::string &dataset
                    , const boost::optional<std::string> &maskDataset
                    , const geo::SrsDefinition &srs
                    , const math::Extents2 &extents
                    , const math::Size2 &size
                    , geo::GeoDataset::Resampling resampling);

private:
    unsigned int processCount_;

    bi::mapped_region mem_;
    ManagedBuffer mb_;
};

GdalWarper::GdalWarper(unsigned int processCount)
    : detail_(std::make_shared<Detail>(processCount))
{}

GdalWarper::Raster
GdalWarper::warpImage(const std::string &dataset
                      , const boost::optional<std::string> &mask
                      , const geo::SrsDefinition &srs
                      , const math::Extents2 &extents
                      , const math::Size2 &size
                      , geo::GeoDataset::Resampling resampling)
{
    return detail().warpImage(dataset, mask, srs, extents, size, resampling);
}

GdalWarper::Raster
GdalWarper::warpMask(const std::string &dataset
                      , const boost::optional<std::string> &mask
                      , const geo::SrsDefinition &srs
                      , const math::Extents2 &extents
                      , const math::Size2 &size
                      , geo::GeoDataset::Resampling resampling)
{
    return detail().warpMask(dataset, mask, srs, extents, size, resampling);
}


namespace {

GdalWarper::Raster allocateMat(ManagedBuffer &mb
                               , const math::Size2 &size, int type)
{
    const auto msize(math::area(size) * CV_ELEM_SIZE(type));

    GdalWarper::Raster mat;
    mat.mat = cv::Mat(size.height, size.width, type
                      , mb.allocate(msize));

    // TODO: hold allocated pointer
    return mat;
}

} // namespace

GdalWarper::Raster
GdalWarper::Detail::warpImage(const std::string &dataset
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

    // grab destination
    auto dstMat(dst.cdata());
    auto type(CV_MAKETYPE(CV_8U, dstMat.channels()));

    auto tile(allocateMat(mb_, size, type));
    dstMat.convertTo(tile.mat, type);

    return tile;
}

GdalWarper::Raster
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
    auto mask(allocateMat(mb_, maskMatSize(cmask), maskMatDataType(cmask)));
    asCvMat(mask.mat, cmask);

    return mask;
}
