#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"

#include "geo/geodataset.hpp"

#include "imgproc/rastermask/cvmat.hpp"

#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/nodeinfo.hpp"

#include "../error.hpp"

#include "./tms-raster.hpp"
#include "./factory.hpp"

#include "browser2d/index.html.hpp"

namespace vr = vadstena::registry;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Config &config
                                      , const Resource &resource)
    {
        return std::make_shared<TmsRaster>(config, resource);
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (resdef::TmsRaster::generator, std::make_shared<Factory>());
});

} // namespace

TmsRaster::TmsRaster(const Config &config, const Resource &resource)
    : Generator(config, resource)
    , definition_(this->resource().definition<resdef::TmsRaster>())
{
    // try to open datasets
    geo::GeoDataset::open(absoluteDataset(definition_.dataset));
    if (definition_.mask) {
        geo::GeoDataset::open(absoluteDataset(*definition_.mask));
    }

    makeReady();
}

void TmsRaster::prepare_impl()
{
    LOG(info2) << "No need to prepare.";
}

vts::MapConfig TmsRaster::mapConfig_impl(ResourceRoot root)
    const
{
    const auto &res(resource());

    vts::MapConfig mapConfig;
    mapConfig.referenceFrame = *res.referenceFrame;

    // this is Tiled service: we have bound layer only
    vr::BoundLayer bl;
    bl.id = res.id.fullId();
    bl.numericId = 0; // no numeric ID
    bl.type = vr::BoundLayer::Type::raster;

    // build url
    bl.url = prependRoot
        (str(boost::format("{lod}-{x}-{y}.%s") % definition_.format)
         , resource(), root);
    bl.maskUrl = prependRoot(std::string("{lod}-{x}-{y}.mask")
                             , resource(), root);

    bl.lodRange = res.lodRange;
    bl.tileRange = res.tileRange;
    bl.credits = res.credits;
    mapConfig.boundLayers.add(bl);

    return mapConfig;
}

Generator::Task TmsRaster::generateFile_impl(const FileInfo &fileInfo
                                             , const Sink::pointer &sink) const
{
    TmsFileInfo fi(fileInfo, config().fileFlags);

    // check for valid tileId
    switch (fi.type) {
    case TmsFileInfo::Type::image:
    case TmsFileInfo::Type::mask:
        if (!checkRanges(resource(), fi.tileId)) {
            sink->error(utility::makeError<NotFound>
                        ("TileId outside of configured range."));
            return {};
        }
        break;

    default: break;
    }

    switch (fi.type) {
    case TmsFileInfo::Type::unknown:
        sink->error(utility::makeError<NotFound>("Unrecognized filename."));
        break;

    case TmsFileInfo::Type::config: {
        std::ostringstream os;
        mapConfig(os, ResourceRoot::none);
        sink->content(os.str(), fi.sinkFileInfo());
        break;
    };

    case TmsFileInfo::Type::support:
        sink->content(fi.support->data, fi.support->size
                      , fi.sinkFileInfo(), false);
        break;

    case TmsFileInfo::Type::image: {
        if (fi.format != definition_.format) {
            sink->error(utility::makeError<NotFound>
                        ("Format <%s> is not supported by this resource (%s)."
                         , fi.format, definition_.format));
            return {};
        }
        return [=]() {
            generateTileImage(fi.tileId, sink);
        };
    }

    case TmsFileInfo::Type::mask:
        return [=]() {
            generateTileMask(fi.tileId, sink);
        };
    }

    return {};
}

namespace {

geo::GeoDataset tileDataSet(const Sink::pointer &sink
                            , const geo::SrsDefinition &srs
                            , const math::Extents2 &extents
                            , const std::string &dataset
                            , const boost::optional<std::string> &mask)
{
    // open source dataset
    auto srcSet(geo::GeoDataset::open(dataset));

    sink->checkAborted();

    // warp
    auto tileSet(geo::GeoDataset::deriveInMemory
                 (srcSet, srs, math::Size2(256, 256)
                  , extents));
    srcSet.warpInto(tileSet, geo::GeoDataset::Resampling::cubic);

    sink->checkAborted();

    if (mask) {
        // open source dataset
        auto srcMaskSet(geo::GeoDataset::open(*mask));

        sink->checkAborted();

        auto maskSet(geo::GeoDataset::deriveInMemory
                     (srcMaskSet, srs, math::Size2(256, 256)
                      , extents));
        srcMaskSet.warpInto(maskSet, geo::GeoDataset::Resampling::cubic);

        tileSet.applyMask(maskSet.cmask());
    }

    return tileSet;
}

} // namespace

void TmsRaster::generateTileImage(const vts::TileId tileId
                                  , const Sink::pointer &sink) const
{
    sink->checkAborted();

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        sink->error(utility::makeError<NotFound>
                    ("TileId outside of valid reference frame tree."));
        return;
    }

    // spatial-division SRS
    const geo::SrsDefinition sds();

    auto tileSet(tileDataSet(sink
                             , vr::Registry::srs(nodeInfo.srs()).srsDef
                             , nodeInfo.extents()
                             , absoluteDataset(definition_.dataset)
                             , absoluteDataset(definition_.mask)));

    if (tileSet.cmask().empty()) {
        // no data -> not found
        sink->error(utility::makeError<NotFound>
                    ("Tile has no valid data."));
        return;
    }

    sink->checkAborted();

    // export
    cv::Mat tile;
    tileSet.exportCvMat(tile, CV_8UC3);

    // serialize
    std::vector<unsigned char> buf;
    // TODO: configurable quality
    cv::imencode(".jpg", tile, buf
                 , { cv::IMWRITE_JPEG_QUALITY, 75 });

    sink->content(buf, Sink::FileInfo(contentType(definition_.format)));
}

void TmsRaster::generateTileMask(const vts::TileId tileId
                                 , const Sink::pointer &sink) const
{
    sink->checkAborted();

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        sink->error(utility::makeError<NotFound>
                    ("TileId outside of valid reference frame tree."));
        return;
    }

    // spatial-division SRS
    const geo::SrsDefinition sds();

    auto tileSet(tileDataSet(sink
                             , vr::Registry::srs(nodeInfo.srs()).srsDef
                             , nodeInfo.extents()
                             , absoluteDataset(definition_.dataset)
                             , absoluteDataset(definition_.mask)));

    if (tileSet.cmask().empty()) {
        // no data -> not found
        sink->error(utility::makeError<NotFound>
                    ("Tile has no valid data."));
        return;
    }

    sink->checkAborted();

    // export
    cv::Mat mask(asCvMat(tileSet.cmask()));

    // serialize
    std::vector<unsigned char> buf;

    // write as png file
    cv::imencode(".png", mask, buf
                 , { cv::IMWRITE_PNG_COMPRESSION, 9 });

    sink->content(buf, Sink::FileInfo(contentType(MaskFormat)));

}

} // namespace generator
