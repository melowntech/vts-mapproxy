#include <new>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/logic/tribool_io.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"
#include "utility/path.hpp"

#include "geometry/mesh.hpp"

#include "geo/coordinates.hpp"

#include "imgproc/rastermask/mappedqtree.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/storage/fstreams.hpp"
#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/tileset/config.hpp"
#include "vts-libs/vts/metatile.hpp"
#include "vts-libs/vts/csconvertor.hpp"
#include "vts-libs/vts/math.hpp"
#include "vts-libs/vts/mesh.hpp"
#include "vts-libs/vts/opencv/navtile.hpp"

#include "../error.hpp"
#include "../support/metatile.hpp"
#include "../support/mesh.hpp"
#include "../support/srs.hpp"
#include "../support/geo.hpp"
#include "../support/grid.hpp"
#include "../support/coverage.hpp"
#include "../support/python.hpp"
#include "../support/tileindex.hpp"

#include "./surface-dem.hpp"
#include "./factory.hpp"
#include "./metatile.hpp"

namespace fs = boost::filesystem;
namespace vr = vadstena::registry;
namespace vs = vadstena::storage;
namespace vts = vadstena::vts;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<SurfaceDem>(params);
    }

    virtual DefinitionBase::pointer definition() {
        return std::make_shared<SurfaceDem::Definition>();
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (Resource::Generator(Resource::Generator::Type::surface
                             , "surface-dem")
         , std::make_shared<Factory>());
});


typedef vts::MetaNode::Flag MetaFlag;
typedef vts::TileIndex::Flag TiFlag;

void parseDefinition(SurfaceDem::Definition &def, const Json::Value &value)
{
    Json::get(def.dataset, value, "dataset");
    if (value.isMember("mask")) {
        std::string s;
        Json::get(s, value, "mask");
        def.mask = s;;
    }

    if (value.isMember("textureLayerId")) {
        Json::get(def.textureLayerId, value, "textureLayerId");
    }

    if (value.isMember("geoidGrid")) {
        std::string s;
        Json::get(s, value, "geoidGrid");
        def.geoidGrid = s;
    }

    if (value.isMember("heightcodingAlias")) {
        def.heightcodingAlias = boost::in_place();
        Json::get(*def.heightcodingAlias, value, "heightcodingAlias");
    }
}

void buildDefinition(Json::Value &value, const SurfaceDem::Definition &def)
{
    value["dataset"] = def.dataset;
    if (def.mask) {
        value["mask"] = def.mask->string();
    }

    if (def.textureLayerId) {
        value["textureLayerId"] = def.textureLayerId;
    }
    if (def.geoidGrid) {
        value["geoidGrid"] = *def.geoidGrid;
    }
    if (def.heightcodingAlias) {
        value["heightcodingAlias"] = *def.heightcodingAlias;
    }
}

void parseDefinition(SurfaceDem::Definition &def
                     , const boost::python::dict &value)
{
    namespace python = boost::python;

    def.dataset = py2utf8(value["dataset"]);

    if (value.has_key("mask")) {
        def.mask = py2utf8(value["mask"]);
    }

    if (value.has_key("textureLayerId")) {
        def.textureLayerId = python::extract<int>(value["textureLayerId"]);
    }

    if (value.has_key("geoidGrid")) {
        def.geoidGrid = py2utf8(value["geoidGrid"]);
    }

    if (value.has_key("heightcodingAlias")) {
        def.heightcodingAlias = py2utf8(value["heightcodingAlias"]);
    }
}

} // namespace

void SurfaceDem::Definition::from_impl(const boost::any &value)
{
    if (const auto *json = boost::any_cast<Json::Value>(&value)) {
        parseDefinition(*this, *json);
    } else if (const auto *py
               = boost::any_cast<boost::python::dict>(&value))
    {
        parseDefinition(*this, *py);
    } else {
        LOGTHROW(err1, Error)
            << "SurfaceDem: Unsupported configuration from: <"
            << value.type().name() << ">.";
    }
}

void SurfaceDem::Definition::to_impl(boost::any &value) const
{
    if (auto *json = boost::any_cast<Json::Value>(&value)) {
        buildDefinition(*json, *this);
    } else {
        LOGTHROW(err1, Error)
            << "SurfaceDem:: Unsupported serialization into: <"
            << value.type().name() << ">.";
    }
}

Changed SurfaceDem::Definition::changed_impl(const DefinitionBase &o) const
{
    const auto &other(o.as<Definition>());

    if (dataset != other.dataset) { return Changed::yes; }
    if (mask != other.mask) { return Changed::yes; }
    if (textureLayerId != other.textureLayerId) { return Changed::yes; }
    if (geoidGrid != other.geoidGrid) { return Changed::yes; }

    return Changed::no;
}

SurfaceDem::SurfaceDem(const Params &params)
    : SurfaceBase(params)
    , definition_(resource().definition<Definition>())
    , dataset_(absoluteDataset(definition_.dataset + "/dem"))
    , maskTree_(absoluteDatasetRf(definition_.mask))
{
    try {
        auto indexPath(filePath(vts::File::tileIndex));
        auto propertiesPath(filePath(vts::File::config));
        if (fs::exists(indexPath) && fs::exists(propertiesPath)) {
            // both paths exist -> ok
            vts::tileset::loadTileSetIndex(index_, indexPath);
            properties_ = vts::tileset::loadConfig(propertiesPath);
            makeReady();

            // remember dem
            addToRegistry();
            return;
        }
    } catch (const std::exception &e) {
        // not ready
    }
    LOG(info1) << "Generator for <" << id() << "> not ready.";
}

SurfaceDem::~SurfaceDem()
{
    removeFromRegistry();
}

void SurfaceDem::prepare_impl(Arsenal&)
{
    LOG(info2) << "Preparing <" << id() << ">.";

    const auto &r(resource());

    // try to open datasets
    auto dataset(geo::GeoDataset::open(dataset_));
    auto datasetMin(geo::GeoDataset::open(dataset_ + ".min"));
    auto datasetMax(geo::GeoDataset::open(dataset_ + ".max"));

    // build properties
    properties_ = {};
    properties_.id = r.id.fullId();
    properties_.referenceFrame = r.referenceFrame->id;
    properties_.credits = asIntSet(r.credits);
    if (definition_.textureLayerId) {
        properties_.boundLayers.insert(definition_.textureLayerId);
    }
    // position ???
    // keep driverOptions empty -> no driver
    properties_.lodRange = r.lodRange;
    properties_.tileRange = r.tileRange;

    prepareTileIndex(index_
                     , (absoluteDataset(definition_.dataset)
                        + "/tiling." + r.id.referenceFrame)
                     , r, true, maskTree_);

    // save it all
    vts::tileset::saveConfig(filePath(vts::File::config), properties_);
    vts::tileset::saveTileSetIndex(index_, filePath(vts::File::tileIndex));

    addToRegistry();
}

void SurfaceDem::addToRegistry()
{
    demRegistry().add(DemRegistry::Record
                      (DemRegistry::Id(referenceFrameId(), id().fullId())
                       , dataset_, id()));
    if (definition_.heightcodingAlias) {
        demRegistry().add(DemRegistry::Record
                          (DemRegistry::Id(referenceFrameId()
                                           , *definition_.heightcodingAlias)
                           , dataset_, id()));
    }
}

void SurfaceDem::removeFromRegistry()
{
    demRegistry().remove(DemRegistry::Id(referenceFrameId(), id().fullId()));
    if (definition_.heightcodingAlias) {
        demRegistry().remove
            (DemRegistry::Id(referenceFrameId()
                             , *definition_.heightcodingAlias));
    }
}

vts::MapConfig SurfaceDem::mapConfig_impl(ResourceRoot root) const
{
    auto mc(vts::mapConfig
            (properties_, resource().registry, vts::ExtraTileSetProperties()
             , prependRoot(fs::path(), resource(), root)));

    // force 2d interface existence
    mc.surfaces.front().has2dInterface = true;

    // look down
    mc.position.orientation = { 0.0, -90.0, 0.0 };

    // take Y size of reference frame's 3D extents extents
    mc.position.verticalExtent
        = math::size(referenceFrame().division.extents).height;

    // quite wide angle camera
    mc.position.verticalFov = 90;

    return mc;
}

namespace {

typedef vs::Range<double> HeightRange;

} // namespace

void SurfaceDem::generateMetatile(const vts::TileId &tileId
                                  , Sink &sink
                                  , const SurfaceFileInfo &fi
                                  , Arsenal &arsenal) const
{
    sink.checkAborted();

    if (!index_.meta(tileId)) {
        sink.error(utility::makeError<NotFound>("Metatile not found."));
        return;
    }

    auto metatile(generateMetatileImpl(tileId, sink, arsenal));

    // write metatile to stream
    std::ostringstream os;
    metatile.save(os);
    sink.content(os.str(), fi.sinkFileInfo());
}

vts::MetaTile
SurfaceDem::generateMetatileImpl(const vts::TileId &tileId
                                 , Sink &sink
                                 , Arsenal &arsenal) const
{
    return metatileFromDem(tileId, sink, arsenal, resource()
                           , index_.tileIndex, dataset_, definition_.geoidGrid
                           , maskTree_);
}

namespace {

inline bool validSample(double value)
{
    return (value >= -1e6);
}

class DemSampler {
public:
    /** Samples point in dem. Dilates by one pixel if pixel is invalid.
     *
     *  Pixels masked by external mask are not valid (i.e. the mask must be
     *  dilated by 1 pixel beforehand)
     */
    DemSampler(const cv::Mat &dem, const vts::NodeInfo::CoverageMask &mask)
        : dem_(dem), mask_(mask)
    {}

    bool operator()(int i, int j, double &h) const {
        // ignore masked-out pixels
        if (!mask_.get(i, j)) { return false; }

        h = dem_.at<double>(j, i);
        if (validSample(h)) { return true; }

        h = 0;
        int count(0);

        for (int jj(-1); jj <= +1; ++jj) {
            for (int ii(-1); ii <= +1; ++ii) {
                // ignore current point
                if (!(ii || jj)) { continue; }

                auto x(i + ii), y(j + jj);
                // check bounds
                if ((x < 0) || (x >= dem_.cols)
                    || (y < 0) || (y >= dem_.rows))
                    { continue; }

                // ingore masked-out pixels
                if (!mask_.get(x, y)) { continue; }

                // sample pixel and use if valid
                auto v(dem_.at<double>(y, x));
                if (!validSample(v)) { continue; }

                h += v;
                ++count;
            }
        }

        if (!count) { return false; }
        h /= count;

        return true;
    }

private:
    cv::Mat dem_;
    const vts::NodeInfo::CoverageMask &mask_;
};

} // namespace

vts::Mesh SurfaceDem::generateMeshImpl(const vts::NodeInfo &nodeInfo
                                       , Sink &sink
                                       , const SurfaceFileInfo&
                                       , Arsenal &arsenal
                                       , bool withMask) const
{
    const int samplesPerSide(128);
    const TileFacesCalculator tileFacesCalculator;

    sink.checkAborted();

    /** warp input dataset as a DEM, with optimized size
     */
    auto dem(arsenal.warper.warp
             (GdalWarper::RasterRequest
              (GdalWarper::RasterRequest::Operation::demOptimal
               , dataset_
               , nodeInfo.srsDef(), nodeInfo.extents()
               , math::Size2(samplesPerSide, samplesPerSide))
              , sink));

    sink.checkAborted();

    // grab size of computed matrix, minus one to get number of edges
    math::Size2 size(dem->cols - 1, dem->rows - 1);

    // generate coverage
    auto coverage(generateCoverage(dem->cols - 1, nodeInfo, maskTree_
                                   , vts::NodeInfo::CoverageType::grid));

    DemSampler ds(*dem, coverage);

    // generate mesh
    auto meshInfo(meshFromNode(nodeInfo, size
                               , [&](int i, int j, double &h) -> bool
    {
        return ds(i, j, h);
    }));
    auto &lm(std::get<0>(meshInfo));

    // simplify
    simplifyMesh(lm, nodeInfo, tileFacesCalculator);

    // and add skirt
    addSkirt(lm, nodeInfo);

    // generate VTS mesh
    vts::Mesh mesh(false);
    if (!lm.vertices.empty()) {
        // local mesh is valid -> add as a submesh into output mesh
        auto &sm(addSubMesh(mesh, lm, nodeInfo, definition_.geoidGrid));
        if (definition_.textureLayerId) {
            sm.textureLayer = definition_.textureLayerId;
        }

        if (withMask) {
            // we are returning full mesh file -> generate coverage mask
            meshCoverageMask
                (mesh.coverageMask, lm, nodeInfo, std::get<1>(meshInfo));
        }
    }

    return mesh;
}

void SurfaceDem::generateNavtile(const vts::TileId &tileId
                                 , Sink &sink
                                 , const SurfaceFileInfo &fi
                                 , Arsenal &arsenal) const
{
    sink.checkAborted();

    const auto &rf(referenceFrame());

    if (!index_.tileIndex.navtile(tileId)) {
        sink.error(utility::makeError<NotFound>("No navtile for this tile."));
        return;
    }

    vts::NodeInfo node(rf, tileId);
    if (!node.valid()) {
        sink.error(utility::makeError<NotFound>
                    ("TileId outside of valid reference frame tree."));
        return;
    }

    auto metaId(tileId);
    {
        metaId.x &= ~((1 << rf.metaBinaryOrder) - 1);
        metaId.y &= ~((1 << rf.metaBinaryOrder) - 1);
    }

    // suboptimal solution: generate metatile
    auto metatile(generateMetatileImpl(metaId, sink, arsenal));

    const auto &extents(node.extents());
    const auto ts(math::size(extents));

    // sds -> navigation SRS convertor
    auto navConv(sds2nav(node, definition_.geoidGrid));

    sink.checkAborted();

    const auto *metanode(metatile.get(tileId, std::nothrow));
    if (!metanode) {
        sink.error(utility::makeError<NotFound>("Metatile not found."));
    }

    const auto heightRange(metanode->heightRange);

    vts::opencv::NavTile nt;
    auto ntd(nt.data());

    // generate coverage mask in grid coordinates
    auto &coverage(nt.coverageMask()
                   = generateCoverage(ntd.cols - 1, node, maskTree_
                                      , vts::NodeInfo::CoverageType::grid));

    // warp input dataset as a DEM
    auto dem(arsenal.warper.warp
             (GdalWarper::RasterRequest
              (GdalWarper::RasterRequest::Operation::dem
               , dataset_
               , node.srsDef(), node.extents()
               , math::Size2(ntd.cols - 1, ntd.rows -1))
              , sink));

    sink.checkAborted();

    // set height range
    nt.heightRange(vts::NavTile::HeightRange
                   (std::floor(heightRange.min), std::ceil(heightRange.max)));
    DemSampler ds(*dem, coverage);

    // calculate navtile values
    math::Size2f npx(ts.width / (ntd.cols - 1)
                     , ts.height / (ntd.rows - 1));
    double h;
    for (int j(0); j < ntd.rows; ++j) {
        auto y(extents.ur(1) - j * npx.height);
        for (int i(0); i < ntd.cols; ++i) {
            // mask with node's mask
            if (!coverage.get(i, j)) { continue; }

            // try to get sample
            if (!ds(i, j, h)) {
                // masked in dem -> remove from coverage
                coverage.set(i, j, false);
                continue;
            }
            auto z(navConv
                   (math::Point3
                    (extents.ll(0) + i * npx.width, y, h))(2));
            // write
            ntd.at<vts::opencv::NavTile::DataType>(j, i) = z;
        }
    }

    // done
    std::ostringstream os;
    if (fi.raw) {
        // raw navtile -> serialize to on-disk format
        nt.serialize(os);
    } else {
        // just navtile itself
        nt.serializeNavtileProper(os);
    }

    sink.content(os.str(), fi.sinkFileInfo());
}

} // namespace generator
