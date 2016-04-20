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
#include "../metatile.hpp"

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
    , hasMetatiles_(false)
{
    LOG(info1) << "Generator for <" << resource.id << "> not ready.";
}

void TmsRaster::prepare_impl()
{
    LOG(info2) << "Preparing <" << resource().id << ">.";

    // try to open datasets
    auto dataset(geo::GeoDataset::open(absoluteDataset(definition_.dataset)));
    if (definition_.mask) {
        geo::GeoDataset::open(absoluteDataset(*definition_.mask));
        // we have mask dataset -> metatiles exist
        hasMetatiles_ = true;
    } else {
        // no external mask available -> metatiles exist only when dataset has
        // some invalid pixels
        hasMetatiles_ = !dataset.allValid();
    }

    makeReady();
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

    if (hasMetatiles_) {
        bl.metaUrl = prependRoot(std::string("{lod}-{x}-{y}.meta")
                                 , resource(), root);
    }

    bl.lodRange = res.lodRange;
    bl.tileRange = res.tileRange;
    bl.credits = asStringSet(res.credits);
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

    case TmsFileInfo::Type::metatile:
        if (!hasMetatiles_) {
            sink->error(utility::makeError<NotFound>
                        ("This dataset doesn't provide metatiles."));
            return {};
        }
        if (!checkRanges(resource(), fi.tileId, RangeType::lod)) {
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
    }

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

        return[=](GdalWarper &warper)  {
            generateTileImage(fi.tileId, sink, warper);
        };
    }

    case TmsFileInfo::Type::mask:
        return [=](GdalWarper &warper) {
            generateTileMask(fi.tileId, sink, warper);
        };

    case TmsFileInfo::Type::metatile:
        return [=](GdalWarper &warper) {
            generateMetatile(fi.tileId, sink, warper);
        };
    }

    return {};
}

void TmsRaster::generateTileImage(const vts::TileId &tileId
                                  , const Sink::pointer &sink
                                  , GdalWarper &warper) const
{
    sink->checkAborted();

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        sink->error(utility::makeError<NotFound>
                    ("TileId outside of valid reference frame tree."));
        return;
    }

    auto tile(warper.warp(GdalWarper::RasterRequest
                          (GdalWarper::RasterRequest::Operation::image
                           , absoluteDataset(definition_.dataset)
                           , vr::Registry::srs(nodeInfo.srs()).srsDef
                           , nodeInfo.extents()
                           , math::Size2(256, 256)
                           , geo::GeoDataset::Resampling::cubic
                           , absoluteDataset(definition_.mask))
                          , sink));

    // serialize
    std::vector<unsigned char> buf;
    // TODO: configurable quality
    cv::imencode(".jpg", *tile, buf
                 , { cv::IMWRITE_JPEG_QUALITY, 75 });

    sink->content(buf, Sink::FileInfo(contentType(definition_.format)));
}

void TmsRaster::generateTileMask(const vts::TileId &tileId
                                 , const Sink::pointer &sink
                                 , GdalWarper &warper) const
{
    sink->checkAborted();

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        sink->error(utility::makeError<NotFound>
                    ("TileId outside of valid reference frame tree."));
        return;
    }

    auto mask(warper.warp(GdalWarper::RasterRequest
                          (GdalWarper::RasterRequest::Operation::mask
                           , absoluteDataset(definition_.dataset
                                             , definition_.mask)
                           , vr::Registry::srs(nodeInfo.srs()).srsDef
                           , nodeInfo.extents()
                           , math::Size2(256, 256)
                           , geo::GeoDataset::Resampling::cubic)
                          , sink));

    // serialize
    std::vector<unsigned char> buf;
    // write as png file
    cv::imencode(".png", *mask, buf
                 , { cv::IMWRITE_PNG_COMPRESSION, 9 });

    sink->content(buf, Sink::FileInfo(contentType(MaskFormat)));

}

namespace Constants {
    const unsigned int RasterMetatileBinaryOrder(8);
    const math::Size2 RasterMetatileSize(1 << RasterMetatileBinaryOrder
                                         , 1 << RasterMetatileBinaryOrder);
}

namespace MetaFlags {
    constexpr std::uint8_t watertight(0xc0);
    constexpr std::uint8_t available(0x80);
    constexpr std::uint8_t unavailable(0x00);
}

void TmsRaster::generateMetatile(const vts::TileId &tileId
                                 , const Sink::pointer &sink
                                 , GdalWarper &warper) const
{
    sink->checkAborted();

    auto blocks(metatileBlocks
                (resource(), tileId, Constants::RasterMetatileBinaryOrder));

    if (blocks.empty()) {
        sink->error(utility::makeError<NotFound>
                    ("Metatile completely outside of configured range."));
        return;
    }

    cv::Mat metatile(Constants::RasterMetatileSize.width
                     , Constants::RasterMetatileSize.height
                     , CV_8U, cv::Scalar(0));

    // bits to set for watertight tile
    const auto watertightBits
        (definition_.mask
         ? MetaFlags::watertight // detailed mask -> watertight supported
         : MetaFlags::available // no mask -> watertight supported
         );

    for (const auto &block : blocks) {
        const auto &view(block.view);
        math::Size2 bSize(vts::tileRangesSize(view));

        GdalWarper::Raster src;
        if (definition_.mask) {
            // warp detailed mask
            src = warper.warp(GdalWarper::RasterRequest
                              (GdalWarper::RasterRequest::Operation::detailMask
                               , absoluteDataset(*definition_.mask)
                               , vr::Registry::srs(block.srs).srsDef
                               , block.extents, bSize)
                              , sink);
        } else {
            // warp dataset as mask
            src = warper.warp(GdalWarper::RasterRequest
                              (GdalWarper::RasterRequest::Operation::mask
                               , absoluteDataset(definition_.dataset)
                               , vr::Registry::srs(block.srs).srsDef
                               , block.extents, bSize
                               , geo::GeoDataset::Resampling::cubic)
                              , sink);
        }

        // generate metatile content for current block
        math::Point2i origin(view.ll(0) - tileId.x, view.ll(1) - tileId.y);
        math::Point2i end(view.ur(0) - tileId.x, view.ur(1) - tileId.y);
        for (int j(origin(1)), je(end(1)), jj(0); j <= je; ++j, ++jj) {
            for (int i(origin(0)), ie(end(0)), ii(0); i <= ie; ++i, ++ii) {
                const auto &in(src->at<double>(jj, ii));
                auto &out(metatile.at<std::uint8_t>(j, i));
                if (!in) {
                    // black -> not available
                    out = MetaFlags::unavailable;
                } else if (in >= 255) {
                    // white -> available and watertight
                    // -> use computed bits above
                    out = watertightBits;
                } else {
                    // gray -> available but not watetight
                    out = MetaFlags::available;
                }
            }
        }
    }

    // serialize metatile
    std::vector<unsigned char> buf;
    // write as png file
    cv::imencode(".png", metatile, buf
                 , { cv::IMWRITE_PNG_COMPRESSION, 9 });
    sink->content(buf, Sink::FileInfo(contentType(RasterMetatileFormat)));
}

} // namespace generator
