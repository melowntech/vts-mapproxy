#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/replace.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"
#include "utility/format.hpp"

#include "geo/heightcoding.hpp"
#include "geo/srsfactors.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/vts/opencv/navtile.hpp"

#include "../support/python.hpp"
#include "../support/tileindex.hpp"
#include "../support/srs.hpp"

#include "./geodata-vector-tiled.hpp"
#include "./factory.hpp"
#include "./metatile.hpp"

namespace ba = boost::algorithm;

namespace vr = vtslibs::registry;
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

/** NOTICE: increment each time some data-related bug is fixed.
 */
int GeneratorRevision(0);

} // namespace

GeodataVectorTiled::GeodataVectorTiled(const Params &params)
    : GeodataVectorBase(params, true)
    , definition_(this->resource().definition<Definition>())
    , dem_(absoluteDataset(definition_.dem.dataset + "/dem")
           , definition_.dem.geoidGrid)
    , effectiveGsdArea_(), effectiveGsdAreaComputed_(false)
    , tileFile_(definition_.dataset)
    , physicalSrs_
      (vr::system.srs(resource().referenceFrame->model.physicalSrs))
    , index_(resource().referenceFrame->metaBinaryOrder)
{
    {
        // open dataset and get descriptor + metadata
        auto ds(geo::GeoDataset::open(dem_.dataset));
        demDescriptor_ = ds.descriptor();

        auto md(ds.getMetadata("vts"));

        if (auto effectiveGSD = md.get<double>("effectiveGSD")) {
            effectiveGsdArea_ = (*effectiveGSD * *effectiveGSD);

            LOG(info2)
                << "<" << id() << ">: using configured effective GSD area of "
                << definition_.dem.dataset << ": "
                << effectiveGsdArea_ << " m2.";
        } else {
            auto esize(math::size(demDescriptor_.extents));
            auto srs(demDescriptor_.srs.reference());
            if (srs.IsGeographic()) {
                // geographic coordinates system -> convert degrees to meters
                double a(srs.GetSemiMajor());
                // double b(srs.GetSemiMinor());

                esize.width *= (a * M_PI / 180.0);

                // TODO: compute length of arc between extents.ll(1) nad
                // extents.ur(1) on the ellipsoid
                // for now, use same calculation as on the equator
                esize.height *= (a * M_PI / 180.0);
            }

            math::Size2f px(esize.width / demDescriptor_.size.width
                            , esize.height / demDescriptor_.size.height);

            effectiveGsdArea_ = math::area(px);
            effectiveGsdAreaComputed_ = true;

            LOG(info2)
                << id() << ": using computed effective GSD area "
                << definition_.dem.dataset << ": "
                << effectiveGsdArea_ << " m2.";
        }
    }

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
    geo::GeoDataset::open(dem_.dataset);
    geo::GeoDataset::open(dem_.dataset + ".min");
    geo::GeoDataset::open(dem_.dataset + ".max");

    // prepare tile index
    prepareTileIndex(index_
                     , (absoluteDataset(definition_.dem.dataset)
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
    def.metaUrl = prependRoot
        (utility::format("{lod}-{x}-{y}.meta?gr=%d", GeneratorRevision)
         , resource(), root);
    def.geodataUrl = prependRoot
        (utility::format("{lod}-{x}-{y}.geo?gr=%d&viewspec={viewspec}"
                         , GeneratorRevision)
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
        LOG(info1) << "trying to find surface";
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
                   , index_.tileIndex, dem_.dataset
                   , dem_.geoidGrid
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
        return;
    }

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.productive()) {
        sink.error(utility::makeError<NotFound>
                   ("TileId outside of valid reference frame tree."));
        return;
    }

    const auto tileFile
        (absoluteDataset
         (tileFile_(vts::UrlTemplate::Vars
                    (tileId, vts::local(nodeInfo.rootLod(), tileId)))));

    LOG(debug) << "Using geo file: <" << tileFile << ">.";

    // combine all dem datasets and default/fallback dem dataset
    auto datasets(viewspec2datasets(fi.fileInfo.query, dem_));

    geo::heightcoding::Config config;
    config.workingSrs = sds(nodeInfo, dem_.geoidGrid);
    config.outputSrs = physicalSrs_.srsDef;
    config.outputVerticalAdjust = physicalSrs_.adjustVertical();
    config.layers = definition_.layers;
    config.clipWorkingExtents = nodeInfo.extents();
    config.format = definition_.format;

    // heightcode data using warper's machinery
    auto hc(arsenal.warper.heightcode
            (tileFile, datasets.first, config, dem_.geoidGrid
             , sink));

    // force 1 hour max age if not all views from viewspec have been found
    boost::optional<long> maxAge;
    if (!datasets.second) { maxAge = 3600; }

    sink.content(hc->data, hc->size
                 , fi.sinkFileInfo().setMaxAge(maxAge), true);
}

} // namespace generator
