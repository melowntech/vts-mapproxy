#include <new>
#include <algorithm>

#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/device/array.hpp>

#include "imgproc/rastermask/cvmat.hpp"

#include "geo/verticaladjuster.hpp"

#include "vts-libs/vts/opencv/navtile.hpp"

#include "../error.hpp"
#include "../support/geo.hpp"
#include "./operations.hpp"

namespace bio = boost::iostreams;
namespace vr = vadstena::registry;

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
            // skip invalid value
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

namespace {

typedef std::shared_ptr< ::GDALDataset> VectorDataset;

class OptionsWrapper {
public:
    OptionsWrapper() : opts_() {}

    ~OptionsWrapper() { ::CSLDestroy(opts_); }

    operator char**() const { return opts_; }

    OptionsWrapper& operator()(const char *name, const char *value) {
        opts_ = ::CSLSetNameValue(opts_, name, value);
        return *this;
    }

    template <typename T>
    OptionsWrapper& operator()(const char *name, const T &value) {
        return operator()
            (name, boost::lexical_cast<std::string>(value).c_str());
    }

    OptionsWrapper& operator()(const char *name, bool value) {
        return operator()(name, value ? "YES" : "NO");
    }

private:
    char **opts_;
};

VectorDataset openVectorDataset(const std::string &dataset
                                , const OptionsWrapper &openOptions)
{
    auto ds(::GDALOpenEx(dataset.c_str(), (GDAL_OF_VECTOR | GDAL_OF_READONLY)
                         , nullptr, openOptions, nullptr));

    if (!ds) {
        LOGTHROW(err2, std::runtime_error)
            << "Failed to open dataset " << dataset << ".";
    }

    return VectorDataset(static_cast< ::GDALDataset*>(ds)
                         , [](::GDALDataset *ds) { delete ds; });
}

VectorDataset openVectorDataset(const std::string &dataset
                                , geo::heightcoding::Config config)
{
    // open vector dataset
    OptionsWrapper openOptions;
    if (config.clipWorkingExtents) {
        openOptions("@MVT_EXTENTS", *config.clipWorkingExtents);
    }

    if (config.workingSrs) {
        openOptions("@MVT_SRS", *config.workingSrs);
    }

    return openVectorDataset(dataset, openOptions);
}

GdalWarper::Heighcoded*
allocateHc(ManagedBuffer &mb
           , const std::string &data
           , const geo::heightcoding::Metadata &metadata)
{
    // create raw memory to hold block and data
    char *raw(static_cast<char*>
              (mb.allocate(sizeof(GdalWarper::Heighcoded) + data.size())));

    // poiter to output data
    auto *dataPtr(raw + sizeof(GdalWarper::Heighcoded));

    // copy data into block
    std::copy(data.begin(), data.end(), dataPtr);

    // allocate block in raw data block
    return new (raw) GdalWarper::Heighcoded
        (dataPtr, data.size(), metadata);
}

GdalWarper::Heighcoded*
heightcode(ManagedBuffer &mb, const VectorDataset &vds
           , const geo::GeoDataset &rds, geo::heightcoding::Config config
           , const boost::optional<std::string> &geoidGrid = boost::none)
{
    if (geoidGrid) {
        // apply geoid grid to SRS of rasterDs and set to rasterDsSrs
        config.rasterDsSrs = geo::setGeoid(rds.srs(), *geoidGrid);
    }

    std::ostringstream os;
    auto metadata(geo::heightcoding::heightCode(*vds, rds, os, config));

    return allocateHc(mb, os.str(), metadata);
}



geo::GeoDataset fromNavtile(const GdalWarper::Navtile &ni
                            , const ConstBlock &dataBlock)
{
    (void) ni;
    (void) dataBlock;
    vts::opencv::NavTile navtile;

    {
        bio::stream_buffer<bio::array_source>
            buffer(dataBlock.data, dataBlock.data + dataBlock.size);
        std::istream is(&buffer);
        is.exceptions(std::ios::badbit | std::ios::failbit);
        navtile.deserialize(ni.heightRange, is, ni.path);
    }

    // get data and mask
    cv::Mat_<vts::opencv::NavTile::DataType> data(navtile.data());
    auto mask(navtile.coverageMask());

    /** We make various assumptions about SDS and navigation SRS.
     *
     * * both share the vertical coordinate system
     *
     * * if navigation SRS is vertically adjusted then both have the same
     *   horizontal system
     */

    auto navSrs(vr::system.srs(ni.navSrs));
    const math::Size2 size(data.cols, data.rows);

    if (navSrs.adjustVertical()) {
        // unudjust Z-coordinates
        geo::VerticalAdjuster va(navSrs.srsDef);

        auto es(math::size(ni.extents));
        math::Size2f px(es.width / size.width, es.height / size.height);
        math::Point2d origin(ul(ni.extents));
        for (auto j(0), ej(size.height); j != ej; ++j, origin(1) += px.height)
        {
            auto p(origin);
            for (auto i(0), ei(size.width); i != ei; ++i, p(0) += px.width) {
                // get height (as reference)
                auto &h(data(j, i));
                // unadjust point and write again
                try {
                    // ignore projection errors
                    h = va({ p(0), p(1), h }, true)(2);
                } catch (...) {}
            }
        }
    }

    // merge(HCS(SDS), VCS(navSrs)) to get proper vertical system
    const auto dsmSds(geo::merge
                      (vr::system.srs(ni.sdsSrs).srsDef, navSrs.srsDef));

    auto ds(geo::GeoDataset::create
            ({}, dsmSds
             , extentsPlusHalfPixel(ni.extents, size), size
             , geo::GeoDataset::Format::dsm
             (geo::GeoDataset::Format::Storage::memory)
             , geo::NodataValue(-1e6)));

    auto &dsData(ds.data());
    data.convertTo(dsData, dsData.type());
    ds.mask() = mask;
    ds.flush();
    return ds;
}

} // namespace

GdalWarper::Heighcoded*
heightcode(DatasetCache &cache, ManagedBuffer &mb
           , const std::string &vectorDs
           , const std::string &rasterDs
           , geo::heightcoding::Config config
           , const boost::optional<std::string> &geoidGrid)
{
    return heightcode(mb, openVectorDataset(vectorDs, config), cache(rasterDs)
                      , config, geoidGrid);
}

GdalWarper::Heighcoded*
heightcode(DatasetCache&, ManagedBuffer &mb
           , const std::string &vectorDs
           , const GdalWarper::Navtile &navtile
           , const ConstBlock &dataBlock
           , geo::heightcoding::Config config)
{
    return heightcode(mb, openVectorDataset(vectorDs, config)
                      , fromNavtile(navtile, dataBlock), config);
}
