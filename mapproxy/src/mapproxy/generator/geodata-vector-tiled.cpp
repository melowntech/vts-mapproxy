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

/** NOTICE: increment each time some data-related bug is fixed.
 */
int GeneratorRevision(0);

} // namespace

GeodataVectorTiled::GeodataVectorTiled(const Params &params)
    : GeodataVectorBase(params, true)
    , definition_(this->resource().definition<Definition>())
    , demDataset_(absoluteDataset(definition_.demDataset + "/dem"))
    , effectiveGsdArea_(), effectiveGsdAreaComputed_(false)
    , tileUrl_(definition_.dataset)
    , physicalSrs_
      (vr::system.srs(resource().referenceFrame->model.physicalSrs))
    , index_(resource().referenceFrame->metaBinaryOrder)
{
    {
        // open dataset and get descriptor + metadata
        auto ds(geo::GeoDataset::open(demDataset_));
        dem_ = ds.descriptor();

        auto md(ds.getMetadata("vts"));

        if (auto effectiveGSD = md.get<double>("effectiveGSD")) {
            effectiveGsdArea_ = (*effectiveGSD * *effectiveGSD);

            LOG(info2)
                << "<" << id() << ">: using configured effective GSD area of "
                << definition_.demDataset << ": "
                << effectiveGsdArea_ << " m2.";
        } else {
            auto esize(math::size(dem_.extents));
            auto srs(dem_.srs.reference());
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

            math::Size2f px(esize.width / dem_.size.width
                            , esize.height / dem_.size.height);

            effectiveGsdArea_ = math::area(px);
            effectiveGsdAreaComputed_ = true;

            LOG(info2)
                << id() << ": using computed effective GSD area "
                << definition_.demDataset << ": "
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
    def.metaUrl = prependRoot
        (utility::format("{lod}-{x}-{y}.meta?gr=%d", GeneratorRevision)
         , resource(), root);
    def.geodataUrl = prependRoot
        (utility::format("{lod}-{x}-{y}.geo?gr=%d", GeneratorRevision)
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
                   , index_.tileIndex, demDataset_, definition_.geoidGrid
                   , MaskTree(), definition_.displaySize));

    // write metatile to stream
    std::ostringstream os;
    metatile.save(os);
    sink.content(os.str(), fi.sinkFileInfo());
}

namespace {

typedef boost::iterator_range<std::string::const_iterator> SubString;
typedef std::vector<SubString> Args;
typedef std::pair<SubString, SubString> KeyValue;

KeyValue splitArgument(const SubString &arg)
{
    auto b(std::begin(arg));
    auto e(std::end(arg));
    for (auto i(b); i != e; ++i) {
        if (*i == '=') {
            return KeyValue(SubString(b, i), SubString(std::next(i), e));
        }
    }
    return KeyValue(SubString(b, e), SubString());
}

struct NavtileInfo {
    std::string url;
    vts::TileId tileId;
    vts::NavTile::HeightRange heightRange;
};

boost::optional<NavtileInfo> parseNavtileInfo(std::string value)
{
    // url-decode value
    value = utility::urlDecode(value);

    std::vector<std::string> parts;
    ba::split(parts, value, ba::is_any_of(";"));
    if (parts.size() != 4) {
        LOG(warn1) << "Navtile info doesn't have 4 elements.";
        return boost::none;
    }

    try {
        NavtileInfo ni;
        // replace .nav -> .rnavtile
        ni.url = ba::replace_all_copy(parts[0], ".nav", ".rnavtile");
        if (ba::starts_with(ni.url, "//")) {
            // scheme-less URI, use http
            ni.url = "http:" + ni.url;
        }
        ni.tileId = boost::lexical_cast<vts::TileId>(parts[1]);
        ni.heightRange.min = boost::lexical_cast<int>(parts[2]);
        ni.heightRange.max = boost::lexical_cast<int>(parts[3]);
        return ni;
    } catch (const boost::bad_lexical_cast&) {
        LOG(warn1) << "Unable to parse navtile info.";
    }

    return boost::none;
}

boost::optional<NavtileInfo> parseQuery(const std::string &query)
{
    Args args;
    ba::split(args, query, ba::is_any_of("&"), ba::token_compress_on);

    for (auto iargs(args.begin()), eargs(args.end()); iargs != eargs; ++iargs)
    {
        auto kv(splitArgument(*iargs));
        if (ba::equals(kv.first, "navtile")) {
            return parseNavtileInfo(std::string(std::begin(kv.second)
                                                , std::end(kv.second)));
        }
    }
    return boost::none;
}

/** Build navtile descriptor from heightrange.
 */
GdalWarper::Navtile buildNavtile(const vts::NodeInfo &nodeInfo
                                 , const vts::NavTile::HeightRange &heightRange
                                 , const std::string &raw
                                 , const std::string &path)
{
    GdalWarper::Navtile nt;
    nt.path = path;
    nt.raw = raw;
    nt.extents = nodeInfo.extents();
    nt.heightRange = heightRange;
    nt.sdsSrs = nodeInfo.srs();
    nt.navSrs = nodeInfo.referenceFrame().model.navigationSrs;
    return nt;
}

void fetchNavtile(Sink &sink, const GeodataFileInfo &fi, Arsenal &arsenal
                  , const NavtileInfo &ni, const vts::NodeInfo &nodeInfo
                  , const GeodataVectorBase::Definition &definition
                  , const vr::Srs &physicalSrs
                  , const std::string &tileUrl
                  , const std::string &demDataset
                  , const boost::optional<std::string> &geoidGrid)
{
    typedef utility::ResourceFetcher::Query Query;
    typedef utility::ResourceFetcher::MultiQuery MultiQuery;

    arsenal.fetcher.perform
        (Query(ni.url).timeout(5000)
         , [=, &arsenal](MultiQuery &&mq) mutable
    {
        try {
            const auto &q(mq.front());

            geo::heightcoding::Config config;
            config.workingSrs = sds(nodeInfo, definition.geoidGrid);
            config.outputSrs = physicalSrs.srsDef;
            config.outputVerticalAdjust = physicalSrs.adjustVertical();
            config.layers = definition.layers;
            config.clipWorkingExtents = nodeInfo.extents();
            config.format = definition.format;

            // heightcode data using warper's machinery (using fetched navtile)
            auto hc(arsenal.warper.heightcode
                    (tileUrl, buildNavtile(nodeInfo, ni.heightRange
                                           , q.get().data, q.location())
                     , config, demDataset, geoidGrid, sink));
            sink.content(hc->data, hc->size, fi.sinkFileInfo(), true);
        } catch (...) {
            sink.error();
        }
    });
}

double computeNavtileGsdArea(const vts::NodeInfo &ni)
{
    auto esize(math::size(ni.extents()));
    auto size(vts::NavTile::size());
    math::Size2f px(esize.width / size.width
                    , esize.height / size.height);
    auto a(math::area(px));
    return a;
}

bool compareGsdArea(const geo::GeoDataset::Descriptor &dem
                    , double effectiveGsdArea
                    , bool effectiveGsdAreaComputed
                    , const vts::NodeInfo &ni)
{
    // navtile srs def
    const auto tileSrs(ni.srsDef());

    // get navtile center (in tile srs)
    auto tc(math::center(ni.extents()));

    const auto tileFactors(geo::SrsFactors(tileSrs)(tc));

    // compute tile GSD area and apply factors (scale down)
    const auto tileGsdArea(computeNavtileGsdArea(ni)
                           / (tileFactors.parallelScale
                              * tileFactors.meridionalScale));

    if (effectiveGsdAreaComputed) {
        // DEM gsd area is computed, apply factors as well
        const auto factors(geo::SrsFactors(dem.srs, tileSrs)(tc));
        // apply scales to area (scale down)
        effectiveGsdArea /= (factors.parallelScale * factors.meridionalScale);
    }

    // true if tile's GSD area is smaller than DEM's area
    return tileGsdArea < effectiveGsdArea;
}

} // namespace

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

    if (!fi.fileInfo.query.empty()) {
        if (auto ni = parseQuery(fi.fileInfo.query)) {
            LOG(info1) << "Received navtile info: url: <" << ni->url
                       << ">, tileId: " << ni->tileId << ", heightRange: "
                       << ni->heightRange << ".";

            vts::NodeInfo niNodeInfo(referenceFrame(), ni->tileId);
            if (!niNodeInfo.valid()) {
                sink.error(utility::makeError<BadRequest>
                           ("Geo navtile id is invalid."));
                return;
            }

            if (ni->tileId.lod > tileId.lod) {
                sink.error(utility::makeError<BadRequest>
                           ("Geo navtile %s is below this geo tile (%s)."
                            , ni->tileId, tileId));
                return;
            }

            if ((ni->tileId.lod < tileId.lod)
                && (ni->tileId != vts::parent
                    (tileId, tileId.lod - ni->tileId.lod)))
            {
                sink.error(utility::makeError<BadRequest>
                           ("Geo navtile %s is not above this geo tile (%s)."
                            , ni->tileId, tileId));
                return;
            }

            // compare GSD
            if (compareGsdArea(dem_, effectiveGsdArea_
                               , effectiveGsdAreaComputed_, niNodeInfo))
            {
                LOG(info1) << "Using navtile " << ni->tileId
                           << " because it is better than DEM at "
                           << tileId << ".";
                fetchNavtile(sink, fi, arsenal, *ni, niNodeInfo
                             , definition_, physicalSrs_, tileUrl
                             , demDataset_, definition_.geoidGrid);
                return;
            }
            LOG(info1) << "Using DEM because navtile " << ni->tileId
                       << " because is worse than DEM at " << tileId << ".";
        }
    }

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
