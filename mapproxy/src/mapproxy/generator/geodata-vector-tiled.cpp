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
    virtual Generator::pointer create(const Generator::Config &config
                                      , const Resource &resource)
    {
        return std::make_shared<GeodataVectorTiled>(config, resource);
    }

    virtual DefinitionBase::pointer definition() {
        return std::make_shared<GeodataVectorTiled::Definition>();
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

void parseDefinition(GeodataVectorTiled::Definition &def
                     , const Json::Value &value)
{
    Json::get(def.displaySize, value, "displaySize");
}

void buildDefinition(Json::Value &value
                     , const GeodataVectorTiled::Definition &def)
{
    value["displaySize"] = def.displaySize;
}

void parseDefinition(GeodataVectorTiled::Definition &def
                     , const boost::python::dict &value)
{
    def.displaySize = boost::python::extract<int>(value["displaySize"]);
}

} // namespace

void GeodataVectorTiled::Definition::from_impl(const boost::any &value)
{
    GeodataVectorBase::Definition::from_impl(value);

    if (const auto *json = boost::any_cast<Json::Value>(&value)) {
        parseDefinition(*this, *json);
    } else if (const auto *py
               = boost::any_cast<boost::python::dict>(&value))
    {
        parseDefinition(*this, *py);
    } else {
        LOGTHROW(err1, Error)
            << "GeodataVectorBase: Unsupported configuration from: <"
            << value.type().name() << ">.";
    }
}

void GeodataVectorTiled::Definition::to_impl(boost::any &value) const
{
    GeodataVectorBase::Definition::to_impl(value);

    if (auto *json = boost::any_cast<Json::Value>(&value)) {
        buildDefinition(*json, *this);
    } else {
        LOGTHROW(err1, Error)
            << "GeodataVectorBase:: Unsupported serialization into: <"
            << value.type().name() << ">.";
    }
}

bool GeodataVectorTiled::Definition::operator==(const Definition &o) const
{
    if (!GeodataVectorBase::Definition::operator==(o)) {
        return false;
    }

    // display size can change
    return true;
}

GeodataVectorTiled::GeodataVectorTiled(const Config &config
                                       , const Resource &resource)
    : GeodataVectorBase(config, resource, true)
    , definition_(this->resource().definition<Definition>())
    , demDataset_(absoluteDataset(definition_.demDataset + "/dem"))
    , tileUrl_(definition_.dataset)
    , physicalSrs_(vr::system.srs(resource.referenceFrame->model.physicalSrs))
    , index_(resource.referenceFrame->metaBinaryOrder)
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

    LOG(info1) << "Generator for <" << resource.id << "> not ready.";
}

void GeodataVectorTiled::prepare_impl()
{
    LOG(info2) << "Preparing <" << resource().id << ">.";

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

vr::FreeLayer GeodataVectorTiled::freeLayer(ResourceRoot root) const
{
    const auto &res(resource());

    vr::FreeLayer fl;
    fl.id = res.id.fullId();

    // TODO: implement me
    (void) root;

    // done
    return fl;
}

vts::MapConfig GeodataVectorTiled::mapConfig_impl(ResourceRoot root)
    const
{
    const auto &res(resource());

    vts::MapConfig mapConfig;
    mapConfig.referenceFrame = *res.referenceFrame;

    (void) root;
    // TODO: add freelayer

    // // this is Tiled service: we have bound layer only; use remote definition
    // mapConfig.boundLayers.add
    //     (vr::BoundLayer
    //      (res.id.fullId()
    //       , prependRoot(std::string("boundlayer.json"), resource(), root)));

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

    geo::HeightCodingConfig config;
    config.workingSrs = sds(nodeInfo, definition_.geoidGrid);
    config.outputSrs = physicalSrs_.srsDef;
    config.outputVerticalAdjust = physicalSrs_.adjustVertical();
    config.layers = definition_.layers;
    config.clipExtents = nodeInfo.extents();
    config.format = definition_.format;

    // heightcode data using warper's machinery
    auto mb(arsenal.warper.heightcode(tileUrl, demDataset_, config, sink));

    sink.content(mb->data, mb->size, fi.sinkFileInfo(), true);
}

} // namespace generator
