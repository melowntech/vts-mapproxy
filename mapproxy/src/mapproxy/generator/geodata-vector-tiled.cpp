#include "utility/premain.hpp"
#include "utility/raise.hpp"

#include "geo/heightcoding.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "../support/python.hpp"
#include "../support/tileindex.hpp"
#include "../support/srs.hpp"

#include "./geodata-vector-tiled.hpp"
#include "./factory.hpp"
#include "./metatile.hpp"

namespace vr = vadstena::registry;
namespace fs = boost::filesystem;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<GeodataVectorTiled>(params);
    }

    virtual DefinitionBase::pointer definition() {
        return std::make_shared<GeodataVectorBase::Definition>();
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (Resource::Generator(Resource::Generator::Type::geodata
                             , "geodata-vector-tiled")
         , std::make_shared<Factory>());
});

} // namespace

GeodataVectorTiled::GeodataVectorTiled(const Params &params)
    : GeodataVectorBase(params, true)
    , definition_(this->resource().definition<Definition>())
    , demDataset_(absoluteDataset(definition_.demDataset + "/dem"))
    , tileUrl_(definition_.dataset)
    , physicalSrs_
      (vr::system.srs(resource().referenceFrame->model.physicalSrs))
    , index_(resource().referenceFrame->metaBinaryOrder)
{
    try {
        auto indexPath(root() / "tileset.index");
        if (fs::exists(indexPath)) {
            // OK, load
            vts::tileset::loadTileSetIndex(index_, indexPath);
            makeReady();
            return;
        }
    } catch (const std::exception &e) {
        // not ready
    }

    LOG(info1) << "Generator for <" << id() << "> not ready.";
}

void GeodataVectorTiled::prepare_impl(Arsenal&)
{
    LOG(info2) << "Preparing <" << id() << ">.";

    const auto &r(resource());

    // try to open datasets
    geo::GeoDataset::open(demDataset_);
    geo::GeoDataset::open(demDataset_ + ".min");
    geo::GeoDataset::open(demDataset_ + ".max");

    // prepare tile index
    prepareTileIndex(index_
                     , (absoluteDataset(definition_.demDataset)
                        + "/tiling." + r.id.referenceFrame)
                     , r);

    // save it all
    vts::tileset::saveTileSetIndex(index_, root() / "tileset.index");
}

vr::FreeLayer GeodataVectorTiled::freeLayer_impl(ResourceRoot root) const
{
    const auto &res(resource());

    vr::FreeLayer fl;
    fl.id = res.id.fullId();
    fl.type = vr::FreeLayer::Type::geodataTiles;

    auto &def(fl.createDefinition<vr::FreeLayer::GeodataTiles>());
    def.metaUrl = prependRoot(std::string("{lod}-{x}-{y}.meta")
                             , resource(), root);
    def.geodataUrl = prependRoot(std::string("{lod}-{x}-{y}.geo")
                                , resource(), root);
    def.style = definition_.styleUrl;

    def.lodRange = res.lodRange;
    def.tileRange = res.tileRange;
    fl.credits = asInlineCredits(res);

    // done
    return fl;
}

vts::MapConfig GeodataVectorTiled::mapConfig_impl(ResourceRoot root)
    const
{
    const auto &res(resource());

    vts::MapConfig mapConfig;
    mapConfig.referenceFrame = *res.referenceFrame;

    // add free layer into list of free layers
    mapConfig.freeLayers.add
        (vr::FreeLayer
         (res.id.fullId()
          , prependRoot(std::string("freelayer.json"), resource(), root)));

    // add free layer into view
    mapConfig.view.freeLayers[res.id.fullId()];

    if (definition_.introspectionSurface) {
        LOG(info4) << "trying to find surface";
        if (auto other = otherGenerator
            (Resource::Generator::Type::surface
             , addReferenceFrame(*definition_.introspectionSurface
                                 , referenceFrameId())))
        {
            mapConfig.merge(other->mapConfig
                            (resolveRoot(resource(), other->resource())));
        }
    }

    // done
    return mapConfig;
}

void GeodataVectorTiled::generateMetatile(Sink &sink
                                          , const GeodataFileInfo &fi
                                          , Arsenal &arsenal) const
{
    sink.checkAborted();

    if (!index_.meta(fi.tileId)) {
        sink.error(utility::makeError<NotFound>("Metatile not found."));
        return;
    }

    auto metatile(metatileFromDem
                  (fi.tileId, sink, arsenal, resource()
                   , index_.tileIndex, demDataset_, definition_.geoidGrid
                   , MaskTree(), definition_.displaySize));

    // write metatile to stream
    std::ostringstream os;
    metatile.save(os);
    sink.content(os.str(), fi.sinkFileInfo());
}

void GeodataVectorTiled::generateGeodata(Sink &sink
                                         , const GeodataFileInfo &fi
                                         , Arsenal &arsenal) const
{
    const auto &tileId(fi.tileId);
    auto flags(index_.tileIndex.get(tileId));
    if (!vts::TileIndex::Flag::isReal(flags)) {
        sink.error(utility::makeError<NotFound>("No geodata for this tile."));
    }

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        sink.error(utility::makeError<NotFound>
                   ("TileId outside of valid reference frame tree."));
        return;
    }

    const auto tileUrl
        (tileUrl_(vts::UrlTemplate::Vars
                  (tileId, vts::local(nodeInfo.rootLod(), tileId))));

    LOG(debug) << "Using geo file: <" << tileUrl << ">.";

    geo::heightcoding::Config config;
    config.workingSrs = sds(nodeInfo, definition_.geoidGrid);
    config.outputSrs = physicalSrs_.srsDef;
    config.outputVerticalAdjust = physicalSrs_.adjustVertical();
    config.layers = definition_.layers;
    config.clipWorkingExtents = nodeInfo.extents();
    config.format = definition_.format;

    // heightcode data using warper's machinery
    auto hc(arsenal.warper.heightcode(tileUrl, demDataset_, config
                                      , boost::none, sink));

    sink.content(hc->data, hc->size, fi.sinkFileInfo(), true);
}

} // namespace generator
