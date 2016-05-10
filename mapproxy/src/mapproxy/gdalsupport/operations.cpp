#include "imgproc/rastermask/cvmat.hpp"

#include "../error.hpp"
#include "../support/geo.hpp"
#include "./operations.hpp"

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

cv::Mat* warpImage(DatasetCache &cache, ManagedBuffer &mb
                   , const std::string &dataset
                   , const geo::SrsDefinition &srs
                   , const math::Extents2 &extents
                   , const math::Size2 &size
                   , geo::GeoDataset::Resampling resampling
                   , const boost::optional<std::string> &maskDataset)
{
    auto &src(cache(dataset));
    auto dst(geo::GeoDataset::deriveInMemory(src, srs, size, extents));
    src.warpInto(dst, resampling);

    if (dst.cmask().empty()) {
        throw EmptyImage("No valid data.");
    }

    // apply mask set if defined
    if (maskDataset) {
        auto &srcMask(cache(*maskDataset));
        auto dstMask(geo::GeoDataset::deriveInMemory
                     (srcMask, srs, size, extents));
        srcMask.warpInto(dstMask, resampling);
        dst.applyMask(dstMask.cmask());

        if (dst.cmask().empty()) {
            throw EmptyImage("No valid data.");
        }
    }

    // grab destination
    auto dstMat(dst.cdata());
    auto type(CV_MAKETYPE(CV_8U, dstMat.channels()));

    auto *tile(allocateMat(mb, size, type));
    dstMat.convertTo(*tile, type);
    return tile;
}

cv::Mat* warpMask(DatasetCache &cache, ManagedBuffer &mb
                  , const std::string &dataset
                  , const geo::SrsDefinition &srs
                  , const math::Extents2 &extents
                  , const math::Size2 &size
                  , geo::GeoDataset::Resampling resampling)
{
    auto &src(cache(dataset));
    auto dst(geo::GeoDataset::deriveInMemory(src, srs, size, extents));
    src.warpInto(dst, resampling);

    if (dst.cmask().empty()) {
        throw EmptyImage("No valid data.");
    }

    auto &cmask(dst.cmask());
    auto *mask(allocateMat(mb, maskMatSize(cmask), maskMatDataType(cmask)));
    asCvMat(*mask, cmask);
    return mask;
}

cv::Mat* warpDetailMask(DatasetCache &cache, ManagedBuffer &mb
                        , const std::string &dataset
                        , const geo::SrsDefinition &srs
                        , const math::Extents2 &extents
                        , const math::Size2 &size)
{
    // generate metatile from mask dataset
    auto &srcMask(cache(dataset));
    auto dstMask(geo::GeoDataset::deriveInMemory
                 (srcMask, srs, size, extents));

    geo::GeoDataset::WarpOptions wo;
    wo.srcNodataValue = geo::GeoDataset::NodataValue();
    wo.dstNodataValue = geo::GeoDataset::NodataValue();
    srcMask.warpInto(dstMask, geo::GeoDataset::Resampling::average, wo);

    // mask is guaranteed to have single (double) channel
    auto &dstMat(dstMask.cdata());
    auto *tile(allocateMat(mb, size, dstMat.type()));
    dstMat.copyTo(*tile);
    return tile;
}

cv::Mat* warpMinMax(DatasetCache &cache, ManagedBuffer &mb
                         , const std::string &dataset
                         , const geo::SrsDefinition &srs
                         , const math::Extents2 &extents
                         , const math::Size2 &size)
{
    auto &minSrc(cache(dataset + ".min"));
    auto &maxSrc(cache(dataset + ".max"));

    auto minDst(geo::GeoDataset::deriveInMemory
                (minSrc, srs, size, extents, GDT_Float64));
    auto maxDst(geo::GeoDataset::deriveInMemory
                (maxSrc, srs, size, extents, GDT_Float64));

    geo::GeoDataset::WarpOptions wo;
    wo.dstNodataValue = std::numeric_limits<double>::lowest();
    minSrc.warpInto(minDst, geo::GeoDataset::Resampling::minimum, wo);
    maxSrc.warpInto(maxDst, geo::GeoDataset::Resampling::maximum, wo);

    // merge first channel from each matrix into one 3-channel matrix
    const auto &dmin(minDst.cdata());
    const auto &dmax(maxDst.cdata());
    const cv::Mat mats[] = { dmin, dmax };
    int pairs[] = { 0, 0, dmin.channels(), 1 };

    auto *tile(allocateMat(mb, size, CV_64FC2));
    cv::mixChannels(mats, 2, tile, 1, pairs, 2);
    return tile;
}

cv::Mat* warpValueMinMax(DatasetCache &cache, ManagedBuffer &mb
                         , const std::string &dataset
                         , const geo::SrsDefinition &srs
                         , const math::Extents2 &extents
                         , const math::Size2 &size
                         , geo::GeoDataset::Resampling resampling)
{
    // combined result of warped dataset and result of warpMinMax
    auto &src(cache(dataset));
    auto &minSrc(cache(dataset + ".min"));
    auto &maxSrc(cache(dataset + ".max"));

    auto dst(geo::GeoDataset::deriveInMemory
             (src, srs, size, extents, GDT_Float64));
    auto minDst(geo::GeoDataset::deriveInMemory
                (minSrc, srs, size, extents, GDT_Float64));
    auto maxDst(geo::GeoDataset::deriveInMemory
                (maxSrc, srs, size, extents, GDT_Float64));

    geo::GeoDataset::WarpOptions wo;
    wo.dstNodataValue = std::numeric_limits<double>::lowest();
    auto wri(src.warpInto(dst, resampling, wo));
    minSrc.warpInto(minDst, geo::GeoDataset::Resampling::minimum, wo);
    maxSrc.warpInto(maxDst, geo::GeoDataset::Resampling::maximum, wo);

    // std::string ovr("None");
    // if (wri.overview) {
    //     ovr = boost::lexical_cast<std::string>(*wri.overview);
    // }
    // LOG(info4)
    //     << std::fixed
    //     << "gdalwarp -ts " << size.width << " " << size.height
    //     << " -te " << extents.ll(0) << " "  << extents.ll(1)
    //     << " " << extents.ur(0) << " " << extents.ur(1)
    //     << " -r " << wri.resampling
    //     << " -dstnodata " << **wo.dstNodataValue
    //     << " -ovr " << ovr
    //     << " " << dataset;

    // merge first channel from each matrix into one 3-channel matrix
    const auto &d(dst.cdata());
    const auto &dmin(minDst.cdata());
    const auto &dmax(maxDst.cdata());
    const cv::Mat mats[] = { d, dmin, dmax };
    int pairs[] = { 0, 0, d.channels(), 1, d.channels() + dmin.channels(), 2 };

    auto *tile(allocateMat(mb, size, CV_64FC3));
    cv::mixChannels(mats, 3, tile, 1, pairs, 3);
    return tile;
}

cv::Mat* warpDem(DatasetCache &cache, ManagedBuffer &mb
                 , const std::string &dataset
                 , const geo::SrsDefinition &srs
                 , const math::Extents2 &extents
                 , const math::Size2 &requestedSize
                 , bool optimize)
{
    auto &src(cache(dataset));

    // calculate size of dataset
    auto size([&]() -> math::Size2
    {
        if (!optimize) { return requestedSize; }

        auto pxc(tileCircumference(extents, srs, src));
        // 1) divide circumference by 4 to get (average) length of one side
        // 2) use 12 samples per one souce pixel
        // 3) add 1 to cover grid
        // 4) clip result to requesed size and 2
        int samples(std::round(12.0 * pxc / 4.0 + 1));
        return math::Size2
            (std::max(std::min(samples, requestedSize.width), 2)
             , std::max(std::min(samples, requestedSize.height), 2));
    }());

    auto dst(geo::GeoDataset::deriveInMemory(src, srs, size, extents));

    geo::GeoDataset::WarpOptions wo;
    wo.dstNodataValue = std::numeric_limits<double>::lowest();
    auto wri(src.warpInto(dst, geo::GeoDataset::Resampling::dem, wo));
    LOG(info1) << "Warp result: scale=" << wri.scale
               << ", resampling=" << wri.resampling << ".";

    if (dst.cmask().empty()) {
        throw EmptyImage("No valid data.");
    }

    // mask is guaranteed to have single (double) channel
    auto &dstMat(dst.cdata());
    auto *tile(allocateMat(mb, size, dstMat.type()));
    dstMat.copyTo(*tile);
    return tile;
}

cv::Mat* warp(DatasetCache &cache, ManagedBuffer &mb
              , const GdalWarper::RasterRequest &req)
{
    typedef GdalWarper::RasterRequest::Operation Operation;

    switch (req.operation) {
    case Operation::image:
        return warpImage
            (cache, mb, req.dataset, req.srs, req.extents, req.size
             , req.resampling, req.mask);

    case Operation::mask:
        return warpMask
            (cache, mb, req.dataset, req.srs, req.extents, req.size
             , req.resampling);

    case Operation::detailMask:
        return warpDetailMask
            (cache, mb, req.dataset, req.srs, req.extents, req.size);

    case Operation::dem:
    case Operation::demOptimal:
        return warpDem
            (cache, mb, req.dataset, req.srs, req.extents, req.size
             , (req.operation == Operation::demOptimal));

    case Operation::minMax:
        return warpMinMax
            (cache, mb, req.dataset, req.srs, req.extents, req.size);

    case Operation::valueMinMax:
        return warpValueMinMax
            (cache, mb, req.dataset, req.srs, req.extents, req.size
             , req.resampling);
    }
    throw;
}
