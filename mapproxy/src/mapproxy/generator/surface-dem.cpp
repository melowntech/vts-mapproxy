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

#include "./surface-dem.hpp"
#include "./factory.hpp"

namespace fs = boost::filesystem;
namespace vr = vadstena::registry;
namespace vs = vadstena::storage;
namespace vts = vadstena::vts;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Config &config
                                      , const Resource &resource)
    {
        return std::make_shared<SurfaceDem>(config, resource);
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

bool SurfaceDem::Definition::operator==(const Definition &o) const
{
    if (dataset != o.dataset) { return false; }
    if (mask != o.mask) { return false; }
    if (textureLayerId != o.textureLayerId) { return false; }
    if (geoidGrid != o.geoidGrid) { return false; }

    return true;
}

SurfaceDem::SurfaceDem(const Config &config
                       , const Resource &resource)
    : SurfaceBase(config, resource)
    , definition_(this->resource().definition<Definition>())
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
            return;
        }
    } catch (const std::exception &e) {
        // not ready
    }
    LOG(info1) << "Generator for <" << resource.id << "> not ready.";
}

void SurfaceDem::prepare_impl()
{
    LOG(info2) << "Preparing <" << resource().id << ">.";

    const auto &r(resource());

    // try to open datasets
    auto dataset(geo::GeoDataset::open(dataset_));
    auto datasetMin(geo::GeoDataset::open(dataset_ + ".min"));
    auto datasetMax(geo::GeoDataset::open(dataset_ + ".max"));

    // load definition
    vts::TileIndex datasetTiles;
    datasetTiles.load(absoluteDataset(definition_.dataset)
                      + "/tiling." + r.id.referenceFrame);

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

    // grab and reset tile index
    auto &ti(index_.tileIndex);

    // clean tile index
    ti = {};

    // generate tile index from lod/tile ranges
    for (auto lod : r.lodRange) {
        // treat whole lod as a huge metatile and process each block
        // independently
        for (const auto &block
                 : metatileBlocks(resource(), vts::TileId(lod), lod, true))
        {
            LOG(info1) << "Generating tile index LOD <" << lod
                       << ">: ancestor: "
                       << block.commonAncestor.nodeId()
                       << ", block: " << block.view << ".";

            if (block.valid() && in(lod, r.lodRange)) {
                TiFlag::value_type flags(TiFlag::mesh);

                if (lod == r.lodRange.min) {
                    // force navtile in topmost lod
                    flags |= TiFlag::navtile;
                }

                // set current block to computed value
                ti.set(lod, block.view, flags);
            }
        }
    }

    // and clip with dataset tiles
    {
        auto combiner([&](TiFlag::value_type o, TiFlag::value_type n)
                      -> TiFlag::value_type
        {
            if (!o || !n) {
                // no intersection -> nothing
                return 0;
            }

            // intersection -> merge flags
            return o | n;
        });

        ti.combine(datasetTiles, combiner, r.lodRange);
    }

    // finally clip everything by mask tree if present
    if (maskTree_) {
        /** TODO: RF partial nodes should be handled differently
         */
        const auto treeDepth(maskTree_.depth());
        for (const auto lod : r.lodRange) {
            auto filterByMask([&](MaskTree::Node node, boost::tribool value)
            {
                // valid -> nothing to be done
                if (value) { return; }

                // update node to match lod grid
                node.shift(treeDepth - lod);

                vts::TileRange tileRange
                    (node.x, node.y
                     , node.x + node.size - 1, node.y + node.size - 1);

                if (!value) {
                    // invalid: unset whole quad
                    ti.set(lod, tileRange, TiFlag::none);
                    return;
                }

                // partially covered tile: unset watertight node; tells the
                // implementation that we are removing some values therefore it
                // is not needed to generate tree for lods that do not exist
                ti.update(lod, tileRange
                          , [&](TiFlag::value_type value)
                {
                    // remove watertight flag
                    return (value & ~TiFlag::watertight);
                }, false);
            });

            maskTree_.forEachQuad(filterByMask, MaskTree::Constraints(lod));
        }
    }

    // save it all
    vts::tileset::saveConfig(filePath(vts::File::config), properties_);
    vts::tileset::saveTileSetIndex(index_, filePath(vts::File::tileIndex));
}

vts::MapConfig SurfaceDem::mapConfig_impl(ResourceRoot root) const
{
    auto mc(vts::mapConfig
            (properties_, vts::ExtraTileSetProperties()
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

inline MetaFlag::value_type ti2metaFlags(TiFlag::value_type ti)
{
    MetaFlag::value_type meta(MetaFlag::allChildren);
    if (ti & TiFlag::mesh) {
        meta |= MetaFlag::geometryPresent;
    }
    if (ti & TiFlag::navtile) {
        meta |= MetaFlag::navtilePresent;
    }

    return meta;
}

/** NB: Do Not Change!
 *
 * This constant has huge impact on dataset stability. Changing this value may
 * break data already served to the outer world.
 */
const int metatileSamplesPerTileBinLog(3);

// real size computed from binary logarithm above
const int metatileSamplesPerTile(1 << metatileSamplesPerTileBinLog);

typedef vs::Range<double> HeightRange;

/** One sample in metatile.
 */
struct Sample {
    bool valid;
    math::Point3 value;
    math::Point3 min;
    math::Point3 max;
    HeightRange heightRange;

    Sample() : valid(false), heightRange(HeightRange::emptyRange()) {}
    Sample(const math::Point3 &value, const math::Point3 &min
           , const math::Point3 &max, const HeightRange &heightRange)
        : valid(true), value(value), min(min), max(max)
        , heightRange(heightRange)
    {}
};

const Sample* getSample(const Sample &sample)
{
    return (sample.valid ? &sample : nullptr);
}

const math::Point3* getValue(const Sample *sample)
{
    return ((sample && sample->valid) ? &sample->value : nullptr);
}

const math::Point3* getValue(const Sample &sample)
{
    return (sample.valid ? &sample.value : nullptr);
}

inline bool validSample(double value)
{
    return (value >= -1e6);
}

class ValueMinMaxSampler {
public:
    ValueMinMaxSampler(const GdalWarper::Raster &dem)
        : dem_(dem)
    {}

    boost::optional<cv::Vec3d> operator()(int i, int j) const {
        // first, try exact value
        const auto &v(dem_->at<cv::Vec3d>(j, i));
        if (validSample(v[0])) { return v; }

        // output vector and count of valid samples
        boost::optional<cv::Vec3d> outFull;
        outFull = boost::in_place(0, std::numeric_limits<double>::max()
                                  , std::numeric_limits<double>::lowest());
        auto &out(*outFull);
        int count(0);

        for (int jj(-1); jj <= +1; ++jj) {
            for (int ii(-1); ii <= +1; ++ii) {
                // ignore current point
                if (!(ii || jj)) { continue; }

                auto x(i + ii), y(j + jj);
                // check bounds
                if ((x < 0) || (x >= dem_->cols)
                    || (y < 0) || (y >= dem_->rows))
                    { continue; }

                const auto &v(dem_->at<cv::Vec3d>(y, x));
                if (validSample(v[0])) {
                    out[0] += v[0];
                    out[1] = std::min(out[1], v[1]);
                    out[2] = std::max(out[2], v[2]);
                    ++count;
                }
            }
        }

        if (!count) { return boost::none; }

        out[0] /= count;
        return out;
    }

private:
    GdalWarper::Raster dem_;
};

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
    auto blocks(metatileBlocks(resource(), tileId));

    if (blocks.empty()) {
        utility::raise<NotFound>
            ("Metatile completely outside of configured range.");
    }

    const auto &rf(referenceFrame());

    vts::MetaTile metatile(tileId, rf.metaBinaryOrder);

    for (const auto &block : blocks) {
        const auto &view(block.view);
        auto extents = block.extents;
        const auto es(math::size(extents));
        const math::Size2 bSize(vts::tileRangesSize(view));

        const math::Size2 gridSize
            (bSize.width * metatileSamplesPerTile + 1
             , bSize.height * metatileSamplesPerTile + 1);

        LOG(info1) << "Processing metatile block ["
                   << vts::tileId(tileId.lod, block.view.ll)
                   << ", " << vts::tileId(tileId.lod, block.view.ur)
                   << "], ancestor: " << block.commonAncestor.nodeId()
                   << ", tile offset: " << block.offset
                   << ", size in tiles: " << vts::tileRangesSize(block.view)
                   << ".";

        // warp value intentionally by average filter
        auto dem(arsenal.warper.warp
                 (GdalWarper::RasterRequest
                  (GdalWarper::RasterRequest::Operation::valueMinMax
                   , dataset_
                   , vr::Registry::srs(block.srs).srsDef
                   // add half pixel to warp in grid coordinates
                   , extentsPlusHalfPixel
                   (extents, { gridSize.width - 1, gridSize.height - 1 })
                   , gridSize
                   , geo::GeoDataset::Resampling::dem)
                  , sink));

        sink.checkAborted();

        Grid<Sample> grid(gridSize);

        // tile size in grid and in real SDS
        math::Size2f gts
            (es.width / (metatileSamplesPerTile * bSize.width)
             , es.height / (metatileSamplesPerTile * bSize.height));
        math::Size2f ts(es.width / bSize.width
                        , es.height / bSize.height);

        auto conv(sds2phys(block.commonAncestor, definition_.geoidGrid));
        auto navConv(sds2nav(block.commonAncestor, definition_.geoidGrid));

        // grid mask
        const ShiftMask rfmask(block, metatileSamplesPerTile
                               , maskTree_);

        // fill in grid
        ValueMinMaxSampler vmm(dem);
        for (int j(0), je(gridSize.height); j < je; ++j) {
            auto y(extents.ur(1) - j * gts.height);
            for (int i(0), ie(gridSize.width); i < ie; ++i) {
                // work only with pixels not masked by combined mask
                if (!rfmask(i, j)) { continue; }

                auto value(vmm(i, j));

                // skip out invalid data
                if (!value) { continue; }

                auto x(extents.ll(0) + i * gts.width);

                // compute all 3 world points (value, min, max) and height range
                // in navigation space
                grid(i, j) = {
                    conv(math::Point3(x, y, (*value)[0]))
                    , conv(math::Point3(x, y, (*value)[1]))
                    , conv(math::Point3(x, y, (*value)[2]))
                    // use only z-component from converted points
                    , HeightRange
                    (navConv(math::Point3(x, y, (*value)[1]))(2)
                     , navConv(math::Point3(x, y, (*value)[2]))(2))
                };
            }
        }

        // release shared data
        dem.reset();

        // generate metatile content
        for (int j(0), je(bSize.height); j < je; ++j) {
            for (int i(0), ie(bSize.width); i < ie; ++i) {
                // ID of current tile
                const vts::TileId nodeId
                    (tileId.lod, view.ll(0) + i, view.ll(1) + j);

                // build metanode
                vts::MetaNode node;
                node.flags(ti2metaFlags(index_.tileIndex.get(nodeId)));
                bool geometry(node.geometry());
                bool navtile(node.navtile());

                // compute tile extents and height range
                auto heightRange(HeightRange::emptyRange());
                math::Extents3 te(math::InvalidExtents{});
                double area(0);
                int triangleCount(0);

                // process all node's vertices in grid
                for (int jj(0); jj <= metatileSamplesPerTile; ++jj) {
                    auto yy(j * metatileSamplesPerTile + jj);
                    for (int ii(0); ii <= metatileSamplesPerTile; ++ii) {
                        auto xx(i * metatileSamplesPerTile + ii);

                        const auto *p(getSample(grid(xx, yy)));

                        // update tile extents (if sample valid)
                        if (p) {
                            // update by both minimum and maximum
                            math::update(te, p->min);
                            math::update(te, p->max);
                        }

                        if (geometry && ii && jj) {
                            // compute area of the quad composed of 1 or 2
                            // triangles
                            auto qa(quadArea
                                    (getValue(grid(xx - 1, yy - 1))
                                     , getValue(p)
                                     , getValue(grid(xx - 1, yy))
                                     , getValue(grid(xx, yy - 1))));
                            area += std::get<0>(qa);
                            triangleCount += std::get<1>(qa);
                        }

                        if (p && navtile) {
                            heightRange
                                = vs::unite(heightRange, p->heightRange);
                        }
                    }
                }

                // build children from tile index
                //
                // TODO: let tileindex generate validity flag for all children
                // in this block
                for (const auto &child : vts::children(nodeId)) {
                    node.setChildFromId
                        (child, index_.tileIndex.validSubtree(child));
                }

                // set extents
                node.extents = vr::normalizedExtents(rf, te);

                // build height range
                node.heightRange.min = std::floor(heightRange.min);
                node.heightRange.max = std::ceil(heightRange.max);

                if (!triangleCount) {
                    // reset content flags
                    node.geometry(geometry = false);
                    node.navtile(navtile = false);
                    heightRange = heightRange.emptyRange();
                }

                // calculate texel size
                if (geometry) {
                    // set credits
                    node.updateCredits(resource().credits);

                    node.applyTexelSize(true);

                    // calculate texture size using node mask
                    auto textureArea([&]() -> double
                    {
                        math::Size2 size(metatileSamplesPerTile
                                         , metatileSamplesPerTile);

                        // return scaled coverage; NB: triangle covers half of
                        // pixel so real area is in pixels is half of number of
                        // pixels
                        return ((triangleCount * vr::BoundLayer::tileArea())
                                / (2.0 * math::area(size)));
                    }());

                    // calculate texel size
                    node.texelSize = std::sqrt(area / textureArea);
                }

                // store metata node
                metatile.set(nodeId, node);
            }
        }
    }

    return metatile;
}

namespace {

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
        auto y(extents.ll(1) + j * npx.height);
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
