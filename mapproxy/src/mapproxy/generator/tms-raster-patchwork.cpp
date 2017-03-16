#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"

#include "geo/geodataset.hpp"

#include "imgproc/rastermask/cvmat.hpp"
#include "imgproc/png.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/qtree-rasterize.hpp"
#include "vts-libs/vts/opencv/colors.hpp"

#include "../error.hpp"
#include "../support/metatile.hpp"
#include "../support/tileindex.hpp"

#include "./tms-raster-patchwork.hpp"
#include "./factory.hpp"
#include "../support/python.hpp"

#include "browser2d/index.html.hpp"

namespace fs = boost::filesystem;
namespace bgil = boost::gil;
namespace vr = vtslibs::registry;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<TmsRasterPatchwork>(params);
    }

    virtual DefinitionBase::pointer definition() {
        return std::make_shared<TmsRasterPatchwork::Definition>();
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (Resource::Generator(Resource::Generator::Type::tms
                             , "tms-raster-patchwork")
         , std::make_shared<Factory>());
});

void parseDefinition(TmsRasterPatchwork::Definition &def, const Json::Value &value)
{
    std::string s;

    if (value.isMember("mask")) {
        def.mask = boost::in_place();
        Json::get(*def.mask, value, "mask");
    }

    if (value.isMember("format")) {
        Json::get(s, value, "format");
        try {
            def.format = boost::lexical_cast<RasterFormat>(s);
        } catch (boost::bad_lexical_cast) {
            utility::raise<Json::Error>
                ("Value stored in format is not RasterFormat value");
        }
    }
}

void buildDefinition(Json::Value &value, const TmsRasterPatchwork::Definition &def)
{
    if (def.mask) {
        value["mask"] = *def.mask;
    }
    value["format"] = boost::lexical_cast<std::string>(def.format);
}

void parseDefinition(TmsRasterPatchwork::Definition &def
                     , const boost::python::dict &value)
{
    if (value.has_key("mask")) {
        def.mask = py2utf8(value["mask"]);
    }

    if (value.has_key("format")) {
        try {
            def.format = boost::lexical_cast<RasterFormat>
                (py2utf8(value["format"]));
        } catch (boost::bad_lexical_cast) {
            utility::raise<Error>
                ("Value stored in format is not RasterFormat value");
        }
    }
}

} // namespace

void TmsRasterPatchwork::Definition::from_impl(const boost::any &value)
{
    if (const auto *json = boost::any_cast<Json::Value>(&value)) {
        parseDefinition(*this, *json);
    } else if (const auto *py
               = boost::any_cast<boost::python::dict>(&value))
    {
        parseDefinition(*this, *py);
    } else {
        LOGTHROW(err1, Error)
            << "TmsRasterPatchwork: Unsupported configuration from: <"
            << value.type().name() << ">.";
    }
}

void TmsRasterPatchwork::Definition::to_impl(boost::any &value) const
{
    if (auto *json = boost::any_cast<Json::Value>(&value)) {
        buildDefinition(*json, *this);
    } else {
        LOGTHROW(err1, Error)
            << "TmsRasterPatchwork:: Unsupported serialization into: <"
            << value.type().name() << ">.";
    }
}

Changed TmsRasterPatchwork::Definition::changed_impl(const DefinitionBase &o) const
{
    const auto &other(o.as<Definition>());

    // non-safe changes first
    if (mask != other.mask) { return Changed::yes; }

    // format can change
    if (format != other.format) { return Changed::safely; }

    // not changed
    return Changed::no;
}

TmsRasterPatchwork::TmsRasterPatchwork(const Params &params)
    : Generator(params)
    , definition_(resource().definition<Definition>())
    , hasMetatiles_(false)
{
    // not seen or index-less
    LOG(info1) << "Generator for <" << id() << "> not ready.";
}

void TmsRasterPatchwork::prepare_impl(Arsenal&)
{
    LOG(info2) << "Preparing <" << id() << ">.";

    // try to open datasets
    if (definition_.mask) {
        geo::GeoDataset::open(absoluteDataset(*definition_.mask));
        // we have mask dataset -> metatiles exist
        hasMetatiles_ = true;
    }

    makeReady();
}

vr::BoundLayer TmsRasterPatchwork::boundLayer(ResourceRoot root) const
{
    const auto &res(resource());

    vr::BoundLayer bl;
    bl.id = res.id.fullId();
    bl.numericId = 0; // no numeric ID
    bl.type = vr::BoundLayer::Type::raster;

    // build url
    bl.url = prependRoot
        (str(boost::format("{lod}-{x}-{y}.%s") % definition_.format)
         , resource(), root);
    if (definition_.mask) {
        bl.maskUrl = prependRoot(std::string("{lod}-{x}-{y}.mask")
                                 , resource(), root);
        if (hasMetatiles_) {
            bl.metaUrl = prependRoot(std::string("{lod}-{x}-{y}.meta")
                                     , resource(), root);
        }
    }

    bl.lodRange = res.lodRange;
    bl.tileRange = res.tileRange;
    bl.credits = asInlineCredits(res);

    // done
    return bl;
}

vts::MapConfig TmsRasterPatchwork::mapConfig_impl(ResourceRoot root)
    const
{
    const auto &res(resource());

    vts::MapConfig mapConfig;
    mapConfig.referenceFrame = *res.referenceFrame;

    // this is Tiled service: we have bound layer only; use remote definition
    mapConfig.boundLayers.add
        (vr::BoundLayer
         (res.id.fullId()
          , prependRoot(std::string("boundlayer.json"), resource(), root)));

    return mapConfig;
}

Generator::Task TmsRasterPatchwork::generateFile_impl(const FileInfo &fileInfo
                                                      , Sink &sink) const
{
    TmsFileInfo fi(fileInfo);

    // check for valid tileId
    switch (fi.type) {
    case TmsFileInfo::Type::image:
    case TmsFileInfo::Type::mask:
        if (!checkRanges(resource(), fi.tileId)) {
            sink.error(utility::makeError<NotFound>
                        ("TileId outside of configured range."));
            return {};
        }
        break;

    case TmsFileInfo::Type::metatile:
        if (!hasMetatiles_) {
            sink.error(utility::makeError<NotFound>
                        ("This dataset doesn't provide metatiles."));
            return {};
        }
        if (!checkRanges(resource(), fi.tileId, RangeType::lod)) {
            sink.error(utility::makeError<NotFound>
                        ("TileId outside of configured range."));
            return {};
        }
        break;

    default: break;
    }

    switch (fi.type) {
    case TmsFileInfo::Type::unknown:
        sink.error(utility::makeError<NotFound>("Unrecognized filename."));
        break;

    case TmsFileInfo::Type::config: {
        std::ostringstream os;
        mapConfig(os, ResourceRoot::none);
        sink.content(os.str(), fi.sinkFileInfo());
        break;
    };

    case TmsFileInfo::Type::definition: {
        std::ostringstream os;
        vr::saveBoundLayer(os, boundLayer(ResourceRoot::none));
        sink.content(os.str(), fi.sinkFileInfo());
        break;
    }

    case TmsFileInfo::Type::support:
        sink.content(fi.support->data, fi.support->size
                      , fi.sinkFileInfo(), false);
        break;

    case TmsFileInfo::Type::image: {
        if (fi.format != definition_.format) {
            sink.error(utility::makeError<NotFound>
                        ("Format <%s> is not supported by this resource (%s)."
                         , fi.format, definition_.format));
            return {};
        }

        return[=](Sink &sink, Arsenal &arsenal) {
            generateTileImage(fi.tileId, fi, sink, arsenal);
        };
    }

    case TmsFileInfo::Type::mask:
        return [=](Sink &sink, Arsenal &arsenal)  {
            generateTileMask(fi.tileId, fi, sink, arsenal);
        };

    case TmsFileInfo::Type::metatile:
        return [=](Sink &sink, Arsenal &arsenal) {
            generateMetatile(fi.tileId, fi, sink, arsenal);
        };
    }

    return {};
}

void TmsRasterPatchwork::generateTileImage(const vts::TileId &tileId
                                           , const TmsFileInfo &fi
                                           , Sink &sink
                                           , Arsenal&) const
{
    sink.checkAborted();

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        sink.error(utility::makeError<NotFound>
                    ("TileId outside of valid reference frame tree."));
        return;
    }

    if (!nodeInfo.productive()) {
        sink.error(utility::makeError<EmptyImage>("No valid data."));
        return;
    }

    unsigned long long int colorIndex(tileId.y);
    colorIndex <<= tileId.lod;
    colorIndex += tileId.x;
    colorIndex = colorIndex & 0xff;

    cv::Mat_<cv::Vec3b> tile(vr::BoundLayer::tileHeight
                             , vr::BoundLayer::tileWidth
                             , vts::opencv::palette256vec[colorIndex]);

    // serialize
    std::vector<unsigned char> buf;
    switch (fi.format) {
    case RasterFormat::jpg:
        // TODO: configurable quality
        cv::imencode(".jpg", tile, buf
                     , { cv::IMWRITE_JPEG_QUALITY, 75 });
        break;

    case RasterFormat::png:
        cv::imencode(".png", tile, buf
                     , { cv::IMWRITE_PNG_COMPRESSION, 9 });
        break;
    }

    sink.content(buf, fi.sinkFileInfo());
}

void TmsRasterPatchwork::generateTileMask(const vts::TileId &tileId
                                 , const TmsFileInfo &fi
                                 , Sink &sink
                                 , Arsenal &arsenal) const
{
    sink.checkAborted();

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        sink.error(utility::makeError<NotFound>
                    ("TileId outside of valid reference frame tree."));
        return;
    }

    if (!nodeInfo.productive()) {
        sink.error(utility::makeError<EmptyImage>("No valid data."));
        return;
    }

    if (!definition_.mask) {
        cv::Mat_<std::uint8_t> mask(vr::BoundLayer::tileHeight
                                    , vr::BoundLayer::tileWidth, 255);

        // serialize
        std::vector<unsigned char> buf;
        // write as png file
        cv::imencode(".png", mask, buf
                     , { cv::IMWRITE_PNG_COMPRESSION, 9 });

        // reset max age received from dataset if mask is uded
        sink.content(buf, fi.sinkFileInfo());
        return;
    }

    auto mask(arsenal.warper.warp
              (GdalWarper::RasterRequest
               (GdalWarper::RasterRequest::Operation::mask
                , absoluteDataset(*definition_.mask)
                , nodeInfo.srsDef()
                , nodeInfo.extents()
                , math::Size2(256, 256)
                , geo::GeoDataset::Resampling::cubic)
               , sink));

    sink.checkAborted();

    // serialize
    std::vector<unsigned char> buf;
    // write as png file
    cv::imencode(".png", *mask, buf
                 , { cv::IMWRITE_PNG_COMPRESSION, 9 });

    // reset max age received from dataset if mask is uded
    sink.content(buf, fi.sinkFileInfo());
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

namespace {

void meta2d(const vts::TileIndex &tileIndex, const vts::TileId &tileId
            , const TmsFileInfo &fi, Sink &sink)
{
    bgil::gray8_image_t out(Constants::RasterMetatileSize.width
                            , Constants::RasterMetatileSize.height
                            , bgil::gray8_pixel_t(0x00), 0);
    auto outView(view(out));

    if (const auto *tree = tileIndex.tree(tileId.lod)) {
        const auto parentId
            (vts::parent(tileId, Constants::RasterMetatileBinaryOrder));

        rasterize(*tree, parentId.lod, parentId.x, parentId.y
                  , outView, [&](vts::QTree::value_type flags) -> std::uint8_t
        {
            std::uint8_t out(0);

            if (flags & vts::TileIndex::Flag::mesh) {
                out |= MetaFlags::available;

                if (flags & vts::TileIndex::Flag::watertight) {
                    out |= MetaFlags::watertight;
                }
            }

            return out;
        });
    }

    sink.content(imgproc::png::serialize(out, 9), fi.sinkFileInfo());
}

} // namespace

void TmsRasterPatchwork::generateMetatile(const vts::TileId &tileId
                                 , const TmsFileInfo &fi
                                 , Sink &sink
                                 , Arsenal &arsenal) const
{
    sink.checkAborted();

    auto blocks(metatileBlocks
                (resource(), tileId, Constants::RasterMetatileBinaryOrder));

    if (blocks.empty()) {
        sink.error(utility::makeError<NotFound>
                    ("Metatile completely outside of configured range."));
        return;
    }

    // non-tileindex code
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
        if (!block.commonAncestor.productive()) { continue; }

        const auto &view(block.view);
        math::Size2 bSize(vts::tileRangesSize(view));

        GdalWarper::Raster src;
        // warp detailed mask
        src = arsenal.warper.warp
            (GdalWarper::RasterRequest
             (GdalWarper::RasterRequest::Operation::detailMask
              , absoluteDataset(*definition_.mask)
              , vr::system.srs(block.srs).srsDef
              , block.extents, bSize)
             , sink);
        sink.checkAborted();

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

    // reset max age received from dataset if mask is uded
    sink.content(buf, fi.sinkFileInfo());
}

} // namespace generator
