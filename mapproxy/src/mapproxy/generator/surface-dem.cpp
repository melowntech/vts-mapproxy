#include <new>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"
#include "utility/path.hpp"

#include "geometry/mesh.hpp"

#include "geo/coordinates.hpp"

#include "imgproc/rastermask/mappedqtree.hpp"

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
#include "../support/grid.hpp"

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

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (resdef::SurfaceDem::generator, std::make_shared<Factory>());
});


typedef vts::MetaNode::Flag MetaFlag;
typedef vts::TileIndex::Flag TiFlag;

} // namespace

SurfaceDem::SurfaceDem(const Config &config
                       , const Resource &resource)
    : SurfaceBase(config, resource)
    , definition_(this->resource().definition<resdef::SurfaceDem>())
    , dataset_(absoluteDataset(definition_.dataset + "/dem"))
    , index_(resource.referenceFrame->metaBinaryOrder)
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

    // generate tile index from lod/tile ranges (and TODO: mask)
    for (auto lod : r.lodRange) {
        // treat whole lod as a huge metatile and process each block
        // independently
        for (const auto &block
                 : metatileBlocks(resource(), vts::TileId(lod), lod, true))
        {
            LOG(info1) << "Generating tile index LOD <" << lod
                       << ">: ancestor: "
                       << block.commonAncestor.nodeId()
                       << "block: " << block.view << ".";

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

            // intersectin -> merge flags
            return o | n;
        });

        ti.combine(datasetTiles, combiner, r.lodRange);
    }

    // save it all
    vts::tileset::saveConfig(filePath(vts::File::config), properties_);
    vts::tileset::saveTileSetIndex(index_, filePath(vts::File::tileIndex));
}

vts::MapConfig SurfaceDem::mapConfig_impl(ResourceRoot root)
    const
{
    auto mc(vts::mapConfig
            (properties_, vts::ExtraTileSetProperties()
             , prependRoot(fs::path(), resource(), root)));

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
const int metatileSamplesPerTile(8);

bool special(const vr::ReferenceFrame &referenceFrame
             , const vts::TileId &tileId)
{
    if (const auto *node
        = referenceFrame.find(vts::rfNodeId(tileId), std::nothrow))
    {
        switch (node->partitioning.mode) {
        case vr::PartitioningMode::manual:
            return true;
        default:
            return false;
        }
    }
    return false;
}

struct Sample {
    bool valid;
    math::Point3 value;
    math::Point3 min;
    math::Point3 max;

    Sample() : valid(false) {}
    Sample(const math::Point3 &value, const math::Point3 &min
           , const math::Point3 &max)
        : valid(true), value(value), min(min), max(max)
    {}
};

const math::Point3* getValue(const Sample *sample)
{
    return ((sample && sample->valid) ? &sample->value : nullptr);
}

math::Extents2 extentsPlusHalfPixel(const math::Extents2 &extents
                                    , const math::Size2 &pixels)
{
    auto es(math::size(extents));
    const math::Size2f px(es.width / pixels.width, es.height / pixels.height);
    const math::Point2 hpx(px.width / 2, px.height / 2);
    return math::Extents2(extents.ll - hpx, extents.ur + hpx);
}

} // namespace

void SurfaceDem::generateMetatile(const vts::TileId &tileId
                                       , const Sink::pointer &sink
                                       , const SurfaceFileInfo &fi
                                       , GdalWarper &warper) const
{
    sink->checkAborted();

    if (!index_.meta(tileId)) {
        sink->error(utility::makeError<NotFound>("Metatile not found."));
        return;
    }

    auto blocks(metatileBlocks(resource(), tileId));

    if (blocks.empty()) {
        sink->error(utility::makeError<NotFound>
                    ("Metatile completely outside of configured range."));
        return;
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

        auto dem(warper.warp
                 (GdalWarper::RasterRequest
                  (GdalWarper::RasterRequest::Operation::valueMinMax
                   , dataset_
                   , vr::Registry::srs(block.srs).srsDef
                   // add half pixel to warp in grid coordinates
                   , extentsPlusHalfPixel
                   (extents, { gridSize.width - 1, gridSize.height - 1 })
                   , gridSize
                   , geo::GeoDataset::Resampling::minimum)
                  , sink));

        Grid<Sample> grid(gridSize);

        // grid mask
        const ShiftMask mask(block, metatileSamplesPerTile);

        // tile size in grid and in real SDS
        math::Size2f gts
            (es.width / (metatileSamplesPerTile * bSize.width)
             , es.height / (metatileSamplesPerTile * bSize.height));
        math::Size2f ts(es.width / bSize.width
                        , es.height / bSize.height);

        auto conv(sds2phys(block.commonAncestor, definition_.geoidGrid));
        auto navConv(sds2nav(block.commonAncestor, definition_.geoidGrid));

        auto sample([&](int i, int j) -> cv::Vec3d
        {
            // first, try exact value
            const auto &v(dem->at<cv::Vec3d>(j, i));
            if (v[0] >= -1e6) { return v; }

            // output vector and count of valid samples
            cv::Vec3d out(0, std::numeric_limits<double>::max()
                          , std::numeric_limits<double>::lowest());
            int count(0);

            for (int jj(-1); jj <= +1; ++jj) {
                for (int ii(-1); ii <= +1; ++ii) {
                    // ignore current point
                    if (!(ii || jj)) { continue; }

                    auto x(i + ii), y(j + jj);
                    // check bounds
                    if ((x < 0) || (x >= dem->cols)
                        || (y < 0) || (y >= dem->rows))
                        { continue; }

                    const auto &v(dem->at<cv::Vec3d>(y, x));
                    if (v[0] >= -1e6) {
                        out[0] += v[0];
                        out[1] = std::min(out[1], v[1]);
                        out[2] = std::max(out[2], v[2]);
                        ++count;
                    }
                }
            }

            if (!count) {
                return { std::numeric_limits<double>::lowest()
                        , std::numeric_limits<double>::lowest()
                        , std::numeric_limits<double>::lowest() };
            }

            out[0] /= count;
            return out;
        });

        // fill in grid
        // TODO: use mask
        for (int j(0), je(gridSize.height); j < je; ++j) {
            auto y(extents.ur(1) - j * gts.height);
            for (int i(0), ie(gridSize.width); i < ie; ++i) {
                // work only with non-masked pixels
                if (!mask(i, j)) { continue; }

                auto value(sample(i, j));

                // skip out invalid data
                if (value[0] < -1e6) { continue; }

                auto x(extents.ll(0) + i * gts.width);

                if ((value[0] < value[1]) || (value[0] > value[2])) {
                    LOG(info4) << "Out of bounds ("
                               << i << ", " << j
                               << "): " << value[1]
                               << " - " << value[0]
                               << " - " << value[2] << ".";
                }
                // compute all 3 world points (value, min, max)
                grid(i, j) = {
                    conv(math::Point3(x, y, value[0]))
                    , conv(math::Point3(x, y, value[1]))
                    , conv(math::Point3(x, y, value[2]))
                };
            }
        }

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
                auto heightRange(vs::Range<double>::emptyRange());
                math::Extents3 te(math::InvalidExtents{});
                double area(0);
                int triangleCount(0);

                // process all node's vertices in grid
                for (int jj(0); jj <= metatileSamplesPerTile; ++jj) {
                    auto yy(j * metatileSamplesPerTile + jj);
                    for (int ii(0); ii <= metatileSamplesPerTile; ++ii) {
                        auto xx(i * metatileSamplesPerTile + ii);
                        const auto *p(grid(mask, xx, yy));

                        // update tile extents (if sample valid)
                        if (p && p->valid) {
                            // update by both minimum and maximum
                            math::update(te, p->min);
                            math::update(te, p->max);
                        }

                        if (geometry && ii && jj) {
                            // compute area of the quad composed of 1 or 2
                            // triangles
                            auto qa(quadArea
                                    (getValue(grid(mask, xx - 1, yy - 1))
                                     , getValue(p)
                                     , getValue(grid(mask, xx - 1, yy))
                                     , getValue(grid(mask, xx, yy - 1))));
                            area += std::get<0>(qa);
                            triangleCount += std::get<1>(qa);
                        }

                        if (p && navtile) {
                            // TODO: implement me
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
                }

                // calculate texel size
                if (geometry) {
                    // set credits
                    node.updateCredits(resource().credits);

                    node.applyTexelSize(true);

                    // calculate texture size using node mask
                    auto textureArea([&]() -> double
                    {
                        // ancestor is full -> we are full as well
                        if (!block.commonAncestor.partial()) {
                            return vr::BoundLayer::tileArea();
                        }

                        // partial node: use triangle count to calculate
                        // percentage of texture
                        math::Size2 size(metatileSamplesPerTile
                                         , metatileSamplesPerTile);

                        // return scaled coverage; NB: triangle covers hald of
                        // pixel so real area is in pixels is half of number of
                        // pixels
                        return ((triangleCount * vr::BoundLayer::tileArea())
                                / (2.0 * math::area(size)));
                    }());

                    // well, empty tile as well
                    if (!textureArea) { continue; }

                    // calculate texel size
                    node.texelSize = std::sqrt(area / textureArea);
                }

                // store metata node
                metatile.set(nodeId, node);
            }
        }
    }

    // write metatile to stream
    std::ostringstream os;
    metatile.save(os);
    sink->content(os.str(), fi.sinkFileInfo());
}

void SurfaceDem::generateMesh(const vts::TileId &tileId
                                   , const Sink::pointer &sink
                                   , const SurfaceFileInfo &fi
                                   , GdalWarper &warper) const
{
    // TODO: calculate tile sampling
    const int samplesPerSide(128);
    // const int samplesPerSide(8);
    const int facesPerTile(1500);

    sink->checkAborted();

    const auto &rf(referenceFrame());

    if (!index_.tileIndex.real(tileId)) {
        sink->error(utility::makeError<NotFound>("No mesh for this tile."));
        return;
    }

    vts::NodeInfo nodeInfo(rf, tileId);
    if (!nodeInfo.valid()) {
        sink->error(utility::makeError<NotFound>
                    ("TileId outside of valid reference frame tree."));
        return;
    }

    auto dem(warper.warp
             (GdalWarper::RasterRequest
              (GdalWarper::RasterRequest::Operation::dem
               , dataset_
               , nodeInfo.srsDef()
               , extentsPlusHalfPixel
               (nodeInfo.extents(), { samplesPerSide, samplesPerSide })
               , math::Size2(samplesPerSide + 1, samplesPerSide + 1)
               , geo::GeoDataset::Resampling::lanczos)
              , sink));

    auto hs([&](int i, int j, double &h) -> bool
    {
        h = dem->at<double>(j, i);
        if (h >= -1e6) { return true; }

        h = 0;
        int count(0);

        for (int jj(-1); jj <= +1; ++jj) {
            for (int ii(-1); ii <= +1; ++ii) {
                // ignore current point
                if (!(ii || jj)) { continue; }

                auto x(i + ii), y(j + jj);
                // check bounds
                if ((x < 0) || (x >= dem->cols)
                    || (y < 0) || (y >= dem->rows))
                    { continue; }

                auto v(dem->at<double>(y, x));
                if (v >= -1e6) {
                    h += v;
                    ++count;
                }
            }
        }

        if (!count) { return false; }
        h /= count;

        return true;
    });

    // generate mesh
    auto meshInfo
        (meshFromNode
         (nodeInfo, math::Size2(samplesPerSide, samplesPerSide), hs));
    auto &lm(std::get<0>(meshInfo));

    // simplify
    simplifyMesh(lm, nodeInfo, facesPerTile);

    // and add skirt
    addSkirt(lm, nodeInfo);

    // generate VTS mesh
    vts::Mesh mesh;
    auto &sm(addSubMesh(mesh, lm, nodeInfo, definition_.geoidGrid));
    if (definition_.textureLayerId) {
        sm.textureLayer = definition_.textureLayerId;
    }

    if (fi.raw) {
        // we are returning full mesh file -> generate coverage mask
        meshCoverageMask
            (mesh.coverageMask, lm, nodeInfo, std::get<1>(meshInfo));
    }

    // write mesh (only mesh!) to stream
    std::ostringstream os;
    if (fi.raw) {
        vts::saveMesh(os, mesh);
    } else {
        vts::saveMeshProper(os, mesh);
    }

    sink->content(os.str(), fi.sinkFileInfo());
}

void SurfaceDem::generateNavtile(const vts::TileId &tileId
                                      , const Sink::pointer &sink
                                      , const SurfaceFileInfo &fi
                                      , GdalWarper&) const
{
    sink->checkAborted();

    const auto &rf(referenceFrame());

    if (!index_.tileIndex.navtile(tileId)) {
        sink->error(utility::makeError<NotFound>("No navtile for this tile."));
        return;
    }

    vts::NodeInfo nodeInfo(rf, tileId);
    if (!nodeInfo.valid()) {
        sink->error(utility::makeError<NotFound>
                    ("TileId outside of valid reference frame tree."));
        return;
    }

    const auto &extents(nodeInfo.extents());
    const auto ts(math::size(extents));

    // sds -> navigation SRS convertor
    auto navConv(sds2nav(nodeInfo, definition_.geoidGrid));

    // first, calculate height range in the same way as is done in metatile
    auto heightRange(vs::Range<double>::emptyRange());
    {
        // create node doverage
        const auto coverage(nodeInfo.coverageMask
                            (vts::NodeInfo::CoverageType::grid
                             , math::Size2(metatileSamplesPerTile + 1
                                           , metatileSamplesPerTile + 1), 1));
        // grid pixel size
        math::Size2f gpx
            (ts.width / (metatileSamplesPerTile + 1)
             , ts.height / (metatileSamplesPerTile + 1));
        for (int j(0); j <= metatileSamplesPerTile; ++j) {
            auto y(extents.ll(1) + j * gpx.height);
            for (int i(0); i <= metatileSamplesPerTile; ++i) {
                if (!coverage.get(i, j)) { continue; }
                auto z(navConv
                   (math::Point3
                    (extents.ll(0) + i * gpx.width, y, 0.0))(2));
                update(heightRange, z);
            }
        }
    }

    // calculate navtile values
    vts::opencv::NavTile nt;
    auto ntd(nt.data());
    // generate coverage mask in grid coordinates
    auto &coverage(nt.coverageMask() = nodeInfo.coverageMask
                   (vts::NodeInfo::CoverageType::grid
                    , math::Size2(ntd.cols, ntd.rows), 1));

    // set height range
    nt.heightRange(vts::NavTile::HeightRange
                   (std::floor(heightRange.min), std::ceil(heightRange.max)));
    math::Size2f npx(ts.width / ntd.cols, ts.height / ntd.rows);
    for (int j(0); j < ntd.rows; ++j) {
        auto y(extents.ll(1) + j * npx.height);
        for (int i(0); i < ntd.cols; ++i) {
            // mask with node's mask
            if (!coverage.get(i, j)) { continue; }
            auto z(navConv
                   (math::Point3
                    (extents.ll(0) + i * npx.width, y, 0.0))(2));
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

    sink->content(os.str(), fi.sinkFileInfo());
}

} // namespace generator
