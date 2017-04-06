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

#include "../error.hpp"
#include "../support/metatile.hpp"
#include "../support/tileindex.hpp"

#include "./tms-raster.hpp"
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
        return std::make_shared<TmsRaster>(params);
    }

    virtual DefinitionBase::pointer definition() {
        return std::make_shared<TmsRaster::Definition>();
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (Resource::Generator(Resource::Generator::Type::tms, "tms-raster")
         , std::make_shared<Factory>());
});

void parseDefinition(TmsRaster::Definition &def, const Json::Value &value)
{
    std::string s;

    Json::get(def.dataset, value, "dataset");
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

    if (value.isMember("transparent")) {
        Json::get(def.transparent, value, "transparent");
    }
}

void buildDefinition(Json::Value &value, const TmsRaster::Definition &def)
{
    value["dataset"] = def.dataset;
    if (def.mask) {
        value["mask"] = *def.mask;
    }
    value["format"] = boost::lexical_cast<std::string>(def.format);

    value["transparent"] = def.transparent;
}

void parseDefinition(TmsRaster::Definition &def
                     , const boost::python::dict &value)
{
    def.dataset = py2utf8(value["dataset"]);

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

    if (value.has_key("transparent")) {
        def.transparent = boost::python::extract<bool>
            (value["transparent"]);
    }
}

} // namespace

void TmsRaster::Definition::from_impl(const boost::any &value)
{
    if (const auto *json = boost::any_cast<Json::Value>(&value)) {
        parseDefinition(*this, *json);
    } else if (const auto *py
               = boost::any_cast<boost::python::dict>(&value))
    {
        parseDefinition(*this, *py);
    } else {
        LOGTHROW(err1, Error)
            << "TmsRaster: Unsupported configuration from: <"
            << value.type().name() << ">.";
    }
}

void TmsRaster::Definition::to_impl(boost::any &value) const
{
    if (auto *json = boost::any_cast<Json::Value>(&value)) {
        buildDefinition(*json, *this);
    } else {
        LOGTHROW(err1, Error)
            << "TmsRaster:: Unsupported serialization into: <"
            << value.type().name() << ">.";
    }
}

Changed TmsRaster::Definition::changed_impl(const DefinitionBase &o) const
{
    const auto &other(o.as<Definition>());

    // non-safe changes first
    if (dataset != other.dataset) { return Changed::yes; }
    if (mask != other.mask) { return Changed::yes; }

    // transparent can change
    if (transparent != other.transparent) { return Changed::safely; }

    // format can change
    if (format != other.format) { return Changed::safely; }

    // not changed
    return Changed::no;
}

TmsRaster::TmsRaster(const Params &params)
    : Generator(params)
    , definition_(resource().definition<Definition>())
    , hasMetatiles_(false)
{
    const auto indexPath(root() / "tileset.index");
    if (fs::exists(indexPath)) {
        index_ = boost::in_place();
        index_->load(indexPath);
        hasMetatiles_ = true;
        makeReady();
        return;
    }

    // not seen or index-less
    LOG(info1) << "Generator for <" << id() << "> not ready.";
}

TmsRaster::DatasetDesc TmsRaster::dataset_impl() const
{
    if (index_) {
        return { definition_.dataset + "/ophoto" };
    }

    return { definition_.dataset };
}

void TmsRaster::prepare_impl(Arsenal&)
{
    LOG(info2) << "Preparing <" << id() << ">.";

    if (fs::exists(absoluteDataset(definition_.dataset + "/ophoto"))) {
        // complex dataset directory
        // alwas with metatiles
        hasMetatiles_ = true;

        // try to open
        geo::GeoDataset::open
            (absoluteDataset(definition_.dataset + "/ophoto"));

        const auto &r(resource());

        // TODO: use mask if provided; must be in new format

        index_ = boost::in_place();
        prepareTileIndex(*index_
                         , (absoluteDataset(definition_.dataset)
                            + "/tiling." + r.id.referenceFrame)
                         , r);

        index_->save(root() / "tileset.index");

        // done
        makeReady();
        return;
   }

    // try to open datasets
    auto ds(geo::GeoDataset::open(absoluteDataset(dataset().path)));
    if (definition_.mask) {
        geo::GeoDataset::open(absoluteDataset(*definition_.mask));
        // we have mask dataset -> metatiles exist
        hasMetatiles_ = true;
    } else {
        // no external mask available -> metatiles exist only when dataset has
        // some invalid pixels
        hasMetatiles_ = !ds.allValid();
    }

    makeReady();
}

RasterFormat TmsRaster::format() const
{
    return transparent() ? RasterFormat::png : definition_.format;
}

bool TmsRaster::transparent_impl() const
{
    return definition_.transparent;
}

bool TmsRaster::hasMask_impl() const
{
    // we want mask by default
    return true;
}

vr::BoundLayer TmsRaster::boundLayer(ResourceRoot root) const
{
    const auto &res(resource());

    vr::BoundLayer bl;
    bl.id = res.id.fullId();
    bl.numericId = 0; // no numeric ID
    bl.type = vr::BoundLayer::Type::raster;

    // build url
    bl.url = prependRoot
        (str(boost::format("{lod}-{x}-{y}.%s") % format())
         , resource(), root);
    if (hasMask()) {
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
    bl.isTransparent = transparent();

    // done
    return bl;
}

vts::MapConfig TmsRaster::mapConfig_impl(ResourceRoot root)
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

Generator::Task TmsRaster::generateFile_impl(const FileInfo &fileInfo
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
        if (fi.format != format()) {
            sink.error(utility::makeError<NotFound>
                        ("Format <%s> is not supported by this resource (%s)."
                         , fi.format, format()));
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

void TmsRaster::generateTileImage(const vts::TileId &tileId
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

    if (!nodeInfo.productive()
        || (index_ && !vts::TileIndex::Flag::isReal(index_->get(tileId))))
    {
        sink.error(utility::makeError<EmptyImage>("No valid data."));
        return;
    }

    auto ds(dataset());

    auto tile(arsenal.warper.warp
              (GdalWarper::RasterRequest
               (GdalWarper::RasterRequest::Operation::image
                , absoluteDataset(ds.path)
                , nodeInfo.srsDef()
                , nodeInfo.extents()
                , math::Size2(256, 256)
                           , geo::GeoDataset::Resampling::cubic
                , absoluteDataset(definition_.mask))
               , sink));
    sink.checkAborted();

    // serialize
    std::vector<unsigned char> buf;
    switch (fi.format) {
    case RasterFormat::jpg:
        // TODO: configurable quality
        cv::imencode(".jpg", *tile, buf
                     , { cv::IMWRITE_JPEG_QUALITY, 75 });
        break;

    case RasterFormat::png:
        cv::imencode(".png", *tile, buf
                     , { cv::IMWRITE_PNG_COMPRESSION, 9 });
        break;
    }

    sink.content(buf, fi.sinkFileInfo().setMaxAge(ds.maxAge));
}

void TmsRaster::generateTileMask(const vts::TileId &tileId
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

    if (!nodeInfo.productive()
        || (index_ && !vts::TileIndex::Flag::isReal(index_->get(tileId))))
    {
        sink.error(utility::makeError<EmptyImage>("No valid data."));
        return;
    }

    // get dataset
    auto ds(dataset());

    auto mask(arsenal.warper.warp
              (GdalWarper::RasterRequest
               (GdalWarper::RasterRequest::Operation::mask
                , absoluteDataset(ds.path, definition_.mask)
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
    if (definition_.mask) { ds.maxAge = boost::none; }
    sink.content(buf, fi.sinkFileInfo().setMaxAge(ds.maxAge));
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

void TmsRaster::generateMetatile(const vts::TileId &tileId
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

    if (index_) {
        // render tileindex
        meta2d(*index_, tileId, fi, sink);
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

    auto ds(dataset());

    for (const auto &block : blocks) {
        if (!block.commonAncestor.productive()) { continue; }

        const auto &view(block.view);
        math::Size2 bSize(vts::tileRangesSize(view));

        GdalWarper::Raster src;
        if (definition_.mask) {
            // warp detailed mask
            src = arsenal.warper.warp
                (GdalWarper::RasterRequest
                 (GdalWarper::RasterRequest::Operation::detailMask
                  , absoluteDataset(*definition_.mask)
                  , vr::system.srs(block.srs).srsDef
                  , block.extents, bSize)
                 , sink);
        } else {
            // warp dataset as mask
            src = arsenal.warper.warp
                (GdalWarper::RasterRequest
                 (GdalWarper::RasterRequest::Operation::maskNoOpt
                  , absoluteDataset(ds.path)
                  , vr::system.srs(block.srs).srsDef
                  , block.extents, bSize
                  , geo::GeoDataset::Resampling::cubic)
                 , sink);
        }
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
    if (definition_.mask) { ds.maxAge = boost::none; }
    sink.content(buf, fi.sinkFileInfo().setMaxAge(ds.maxAge));
}

} // namespace generator
