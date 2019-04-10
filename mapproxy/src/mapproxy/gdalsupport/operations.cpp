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

#include <new>
#include <algorithm>
#include <cstring>
#include <type_traits>

#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/device/array.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <sqlite3.h>

#include <ogrsf_frmts.h>

#include "utility/gccversion.hpp"

#include "imgproc/rastermask/cvmat.hpp"

#include "geo/verticaladjuster.hpp"
#include "geo/csconvertor.hpp"
#include "geo/srs.hpp"

#include "../error.hpp"
#include "../support/geo.hpp"
#include "operations.hpp"

namespace bio = boost::iostreams;
namespace vr = vtslibs::registry;

namespace {

cv::Mat* allocateMat(ManagedBuffer &mb
                     , const math::Size2 &size, int type)
{
    // calculate sizes
    const auto dataSize(math::area(size) * CV_ELEM_SIZE(type));
    const auto matSize(sizeof(cv::Mat) + dataSize);

    // create raw memory to hold matrix and data
    char *raw(static_cast<char*>
              (mb.allocate_aligned(matSize, alignof(cv::Mat))));

    // allocate matrix in raw data block
    return new (raw) cv::Mat(size.height, size.width, type
                             , raw + sizeof(cv::Mat));
}

inline geo::OptionalNodataValue
asOptNodata(const geo::NodataValue &nodata
            , const geo::NodataValue &dflt = boost::none)
{
    return nodata ? nodata : dflt;
}

cv::Mat* warpImage(DatasetCache &cache, ManagedBuffer &mb
                   , const std::string &dataset
                   , const geo::SrsDefinition &srs
                   , const math::Extents2 &extents
                   , const math::Size2 &size
                   , geo::GeoDataset::Resampling resampling
                   , const boost::optional<std::string> &maskDataset
                   , bool optimize
                   , const geo::NodataValue &nodata)
{
    auto &src(cache(dataset));
    auto dst(geo::GeoDataset::deriveInMemory
             (src, srs, size, extents, boost::none, asOptNodata(nodata)));
    src.warpInto(dst, resampling);

    if (optimize && dst.cmask().empty()) {
        throw EmptyImage("No valid data.");
    }

    // apply mask set if defined
    if (maskDataset) {
        auto &srcMask(cache(*maskDataset));
        auto dstMask(geo::GeoDataset::deriveInMemory
                     (srcMask, srs, size, extents));
        srcMask.warpInto(dstMask, resampling);
        dst.applyMask(dstMask.cmask());

        if (optimize && dst.cmask().empty()) {
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
                  , geo::GeoDataset::Resampling resampling
                  , bool optimize
                  , const geo::NodataValue &nodata)
{
    auto &src(cache(dataset));
    auto dst(geo::GeoDataset::deriveInMemory
             (src, srs, size, extents, boost::none, asOptNodata(nodata)));
    src.warpInto(dst, resampling);

    // fetch mask from dataset (optimized, all valid -> invalid matrix)
    auto m(dst.fetchMask(optimize));
    if (!m.data) {
        // all pixels valid
        throw FullImage("All data valid.");
    }

    if (optimize) {
        auto nonzero(cv::countNonZero(m));
        if (!nonzero) {
            // empty mask -> no valid data
            throw EmptyImage("No valid data.");
        } else if (nonzero == area(size)) {
            // all pixels valid
            throw FullImage("All data valid.");
        }
    }

    auto *mask(allocateMat(mb, size, m.type()));
    m.copyTo(*mask);
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
                         , geo::GeoDataset::Resampling resampling
                         , const geo::NodataValue &nodata)
{
    // combined result of warped dataset and result of warpMinMax
    auto &src(cache(dataset));
    auto &minSrc(cache(dataset + ".min"));
    auto &maxSrc(cache(dataset + ".max"));

    const auto nd(asOptNodata(nodata, ForcedNodata));

    auto dst(geo::GeoDataset::deriveInMemory
             (src, srs, size, extents, GDT_Float32, nd));
    auto minDst(geo::GeoDataset::deriveInMemory
                (minSrc, srs, size, extents, GDT_Float32, nd));
    auto maxDst(geo::GeoDataset::deriveInMemory
                (maxSrc, srs, size, extents, GDT_Float32, nd));

    geo::GeoDataset::WarpOptions warpOptions;
#if GDAL_VERSION_NUM >= 2020000
    // choose finer overview on GDAL>=2.2, since there is something rotten there
    warpOptions.overviewBias = -1;
#endif

    auto wri(src.warpInto(dst, resampling, warpOptions));
    minSrc.warpInto(minDst, geo::GeoDataset::Resampling::minimum
                    , warpOptions);
    maxSrc.warpInto(maxDst, geo::GeoDataset::Resampling::maximum
                    , warpOptions);

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
                 , bool optimize
                 , const geo::NodataValue &nodata)
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
              , asOptNodata(nodata, ForcedNodata)));

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
    case Operation::imageNoOpt:
        return warpImage
            (cache, mb, req.dataset, req.srs, req.extents, req.size
             , req.resampling, req.mask
             , (req.operation == Operation::image), req.nodata);

    case Operation::mask:
    case Operation::maskNoOpt:
        return warpMask
            (cache, mb, req.dataset, req.srs, req.extents, req.size
             , req.resampling, (req.operation == Operation::mask)
             , req.nodata);

    case Operation::detailMask:
        return warpDetailMask
            (cache, mb, req.dataset, req.srs, req.extents, req.size);

    case Operation::dem:
    case Operation::demOptimal:
        return warpDem
            (cache, mb, req.dataset, req.srs, req.extents, req.size
             , (req.operation == Operation::demOptimal)
             , req.nodata);

    case Operation::valueMinMax:
        return warpValueMinMax
            (cache, mb, req.dataset, req.srs, req.extents, req.size
             , req.resampling, req.nodata);
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

    OptionsWrapper& operator()(const std::string &pair) {
        opts_ = ::CSLAddString(opts_, pair.c_str());
        return *this;
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
        const auto code(::CPLGetLastErrorNo());
        if (code == CPLE_OpenFailed) {
            LOGTHROW(err2, EmptyGeoData)
                << "No file found for " << dataset << ".";
        }

        LOGTHROW(err2, std::runtime_error)
            << "Failed to open dataset " << dataset << " ("
            << ::CPLGetLastErrorNo() << ").";
    }

    return VectorDataset(static_cast< ::GDALDataset*>(ds)
                         , [](::GDALDataset *ds) { delete ds; });
}

VectorDataset openVectorDataset(const std::string &dataset
                                , const geo::heightcoding::Config&
                                , const GdalWarper::OpenOptions &openOptions)
{
    OptionsWrapper ow;
    for (const auto &option : openOptions) { ow(option); }
    return openVectorDataset(dataset, ow);
}

GdalWarper::Heightcoded*
allocateHc(ManagedBuffer &mb
           , const std::string &data
           , const geo::heightcoding::Metadata &metadata)
{
    // create raw memory to hold block and data
    char *raw(static_cast<char*>
              (mb.allocate_aligned
               (sizeof(GdalWarper::Heightcoded) + data.size()
                , alignof(GdalWarper::Heightcoded))));

    // poiter to output data
    auto *dataPtr(raw + sizeof(GdalWarper::Heightcoded));

    // copy data into block
    std::copy(data.begin(), data.end(), dataPtr);

    // allocate block in raw data block
    return new (raw) GdalWarper::Heightcoded
        (dataPtr, data.size(), metadata);
}

struct DbError : public std::runtime_error {
    DbError(const std::string &msg) : std::runtime_error(msg) {}
};

class SQLStatement {
public:
    SQLStatement() : stmt_() {}
    ~SQLStatement() { if (stmt_) { ::sqlite3_finalize(stmt_); } }
    operator ::sqlite3_stmt*() { return stmt_; }
    operator ::sqlite3_stmt**() { return &stmt_; }

private:
    ::sqlite3_stmt *stmt_;
};

typedef geo::FeatureLayers::Features::Properties FeatureProperties;
typedef geo::FeatureLayers::Features::Fid Fid;

class EnhanceDatabase {
public:
    EnhanceDatabase(const std::string &path
                    , const std::string &table)
        : path_(path), table_(table), db_()
    {
        check(::sqlite3_open_v2
              (path.c_str(), &db_, SQLITE_OPEN_READONLY, nullptr)
              , "sqlite3_open_v2");

        std::ostringstream os;
        os << "SELECT * FROM `" << table << "` WHERE `id`=?";

        const auto &str(os.str());
        check(::sqlite3_prepare_v2(db_, str.data(), str.size()
                                , select_, nullptr)
              , "sqlite3_prepare");
    }

    ~EnhanceDatabase() { if (db_) { ::sqlite3_close(db_); } }

    void enhance(FeatureProperties &properties, const std::string &id) {
        check(::sqlite3_reset(select_), "sqlite3_reset");
        check(::sqlite3_bind_text(select_, 1, id.data(), id.size(), nullptr)
              , "sqlite3_bind_text");

        switch (auto res = ::sqlite3_step(select_)) {
        case SQLITE_ROW: break;

        case SQLITE_DONE:
            // nothing found
            return;

        default:
            check(res, "sqlite3_step");
        }

        // process all columns
        const int columns(::sqlite3_column_count(select_));
        for (int column(0); column < columns; ++column) {
            const auto name(::sqlite3_column_name(select_, column));
            // skip id itself
            if (!std::strcmp(name, "id")) { continue; }
            const auto *text(reinterpret_cast<const char*>
                             (::sqlite3_column_text(select_, column)));
            const auto size(::sqlite3_column_bytes(select_, column));
            properties.insert(FeatureProperties::value_type
                              (name, std::string(text, size)));
        }
    }

    template <typename T>
    void enhance(FeatureProperties &properties, const T &id) {
        return enhance(properties, boost::lexical_cast<std::string>(id));
    }

private:
    void check(int status, const char *what) const {
        if (status) {
            const char *msg(::sqlite3_errmsg(db_));
            LOGTHROW(err1, DbError)
                << "Sqlite3 operation " << what << " failed: <"
                << msg << "> (file \"" << path_ << "\").";
        }
    }

    const std::string path_;
    const std::string table_;
    ::sqlite3 *db_;
    SQLStatement select_;
};

void enhanceLayer(const LayerEnhancer &enhancer
                   , geo::FeatureLayers::Layer &layer)
{
    EnhanceDatabase db(enhancer.databasePath, enhancer.table);

    // special handling of feature ID
    if (enhancer.key == "#fid") {
        const auto manipulator([&](Fid fid, FeatureProperties &properties)
        {
            db.enhance(properties, fid);
        });
        layer.features.updateProperties(manipulator);
        return;
    }

    // properties
    const auto manipulator([&](Fid, FeatureProperties &properties)
    {
        const auto fkey(properties.find(enhancer.key));
        if (fkey == properties.end()) { return; }
        db.enhance(properties, fkey->second);
    });

    layer.features.updateProperties(manipulator);
}

void enhanceLayers(const LayerEnhancer::map &layerEnancers
                   , geo::FeatureLayers &layers)
{
    for (auto &layer : layers.layers) {
        auto flayerEnancers(layerEnancers.find(layer.name));
        if (flayerEnancers == layerEnancers.end()) { continue; }

        const auto &enhancer(flayerEnancers->second);
        enhanceLayer(enhancer, layer);
    }
}

GdalWarper::Heightcoded*
heightcode(ManagedBuffer &mb, const VectorDataset &vds
           , std::vector<const geo::GeoDataset*> rds
           , geo::heightcoding::Config config
           , const boost::optional<std::string> &geoidGrid
           , const boost::optional<std::string> &vectorGeoidGrid
           , const LayerEnhancer::map &layerEnancers)
{
    if (geoidGrid) {
        // apply geoid grid to SRS of rasterDs and set to rasterDsSrs
        config.rasterDsSrs = geo::setGeoid(rds.back()->srs(), *geoidGrid);
    }

    if (vectorGeoidGrid && vds->GetLayerCount()) {
        if (auto ref = vds->GetLayer(0)->GetSpatialRef()) {
            // set vertical srs
            config.vectorDsSrs
                = geo::SrsDefinition::fromReference
                (geo::setGeoid(*ref, *vectorGeoidGrid));
        }
    }

    if (!layerEnancers.empty()) {
        config.postprocess = [&](geo::FeatureLayers &layers) -> void {
            enhanceLayers(layerEnancers, layers);
        };
    }

    std::ostringstream os;
    auto metadata(geo::heightcoding::heightCode(*vds, rds, os, config));

    return allocateHc(mb, os.str(), metadata);
}

} // namespace

GdalWarper::Heightcoded*
heightcode(DatasetCache &cache, ManagedBuffer &mb
           , const std::string &vectorDs
           , const DemDataset::list &rasterDs
           , geo::heightcoding::Config config
           , const boost::optional<std::string> &vectorGeoidGrid
           , const GdalWarper::OpenOptions &openOptions
           , const LayerEnhancer::map &layerEnancers)
{
    std::vector<const geo::GeoDataset*> rasterDsStack;
    for (const auto &ds : rasterDs) {
        rasterDsStack.push_back(&cache(ds.dataset));
    }

    return heightcode(mb, openVectorDataset(vectorDs, config, openOptions)
                      , rasterDsStack
                      , config, rasterDs.back().geoidGrid
                      , vectorGeoidGrid, layerEnancers);
}
