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
                 (srcMask, srs, size, extents, boost::none
                  , geo::GeoDataset::NodataValue()));

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

const auto ForcedNodata(geo::GeoDataset::NodataValue(-1e10f));

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
             (src, srs, size, extents, GDT_Float32, ForcedNodata));
    auto minDst(geo::GeoDataset::deriveInMemory
                (minSrc, srs, size, extents, GDT_Float32, ForcedNodata));
    auto maxDst(geo::GeoDataset::deriveInMemory
                (maxSrc, srs, size, extents, GDT_Float32, ForcedNodata));

    auto wri(src.warpInto(dst, resampling));
    minSrc.warpInto(minDst, geo::GeoDataset::Resampling::minimum);
    maxSrc.warpInto(maxDst, geo::GeoDataset::Resampling::maximum);

    // combine data
    auto *tile(allocateMat(mb, size, CV_64FC3));
    *tile = cv::Scalar(*ForcedNodata, *ForcedNodata, *ForcedNodata);

    {
        // TODO: use masks (get them as a byte matrices)
        const auto &d(dst.cdata());
        const auto &dmin(minDst.cdata());
        const auto &dmax(maxDst.cdata());

        auto id(d.begin<double>());
        auto idmin(dmin.begin<double>());
        auto idmax(dmax.begin<double>());

        for (auto itile(tile->begin<cv::Vec3d>())
                 , etile(tile->end<cv::Vec3d>());
             itile != etile; ++itile, ++id, ++idmin, ++idmax)
        {
            // skil invalid value
            auto value(*id);
            if (value == ForcedNodata) { continue; }

            auto &sample(*itile);
            sample[0] = value;

            if ((*idmin == ForcedNodata) || (*idmin > value)) {
                // clone value into minimum if minimum is invalid or above value
                sample[1] = value;
            } else {
                // copy min
                sample[1] = *idmin;
            }

            if ((*idmax == ForcedNodata) || (*idmax < value)) {
                // clone value into maximum if maximum is invalid or below value
                sample[2] = value;
            } else {
                // copy max
                sample[2] = *idmax;
            }
        }
    }

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
        // 4) clip result to requesed size and 2
        int samples(std::round(12.0 * pxc / 4.0));
        return math::Size2
            (std::max(std::min(samples, requestedSize.width), 2)
             , std::max(std::min(samples, requestedSize.height), 2));
    }());

    // simulate grid registration
    math::Size2 gridSize(size.width + 1, size.height + 1);
    auto gridExtents(extentsPlusHalfPixel(extents, size));

    // warp in floats
    auto dst(geo::GeoDataset::deriveInMemory
             (src, srs, gridSize, gridExtents, ::GDT_Float32
              , ForcedNodata));

    auto wri(src.warpInto(dst, geo::GeoDataset::Resampling::dem));
    LOG(info1) << "Warp result: scale=" << wri.scale
               << ", resampling=" << wri.resampling << ".";

    // mask is guaranteed to have single (double) channel
    auto &dstMat(dst.cdata());
    auto *tile(allocateMat(mb, gridSize, dstMat.type()));
    dstMat.copyTo(*tile);
    return tile;
}

} // namespace

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

    case Operation::valueMinMax:
        return warpValueMinMax
            (cache, mb, req.dataset, req.srs, req.extents, req.size
             , req.resampling);
    }
    throw;
}
