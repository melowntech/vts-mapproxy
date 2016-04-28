#include <new>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"

#include "geometry/mesh.hpp"

#include "geo/coordinates.hpp"

#include "imgproc/scanconversion.hpp"

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

#include "./surface-spheroid.hpp"
#include "./factory.hpp"

#include "browser2d/index.html.hpp"

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
        return std::make_shared<SurfaceSpheroid>(config, resource);
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (resdef::SurfaceSpheroid::generator, std::make_shared<Factory>());
});

} // namespace

fs::path SurfaceSpheroid::filePath(vts::File fileType) const
{
    switch (fileType) {
    case vts::File::config:
        return root() / "tileset.conf";
    case vts::File::tileIndex:
        return root() / "tileset.index";
    default: break;
    }

    throw utility::makeError<InternalError>("Unsupported file");
}

SurfaceSpheroid::SurfaceSpheroid(const Config &config
                                 , const Resource &resource)
    : Generator(config, resource)
    , definition_(this->resource().definition<resdef::SurfaceSpheroid>())
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

void SurfaceSpheroid::prepare_impl()
{
    LOG(info2) << "Preparing <" << resource().id << ">.";

    const auto &r(resource());

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

    // TODO: spatialDivisionExtents

    // grab and reset tile index
    auto &ti(index_.tileIndex);
    ti = {};

    // build tile index
    for (vts::Lod lod(0); lod <= r.lodRange.max; ++lod) {
        // metatiles everywhere
        vts::TileIndex::Flag::value_type flags
            (vts::TileIndex::Flag::meta);
        if (in(lod, r.lodRange)) {
            // TODO: what about polar caps in melown2015?
            // watertight tiles and navtiles everywhere
            flags |= (vts::TileIndex::Flag::mesh
                      | vts::TileIndex::Flag::watertight
                      | vts::TileIndex::Flag::navtile);
        }
        // set whole LOD to given value
        ti.set(lod, flags);
    }

    // save it all
    vts::tileset::saveConfig(filePath(vts::File::config), properties_);
    vts::tileset::saveTileSetIndex(index_, filePath(vts::File::tileIndex));
}

vts::MapConfig SurfaceSpheroid::mapConfig_impl(ResourceRoot root)
    const
{
    auto mc(vts::mapConfig
            (properties_, vts::ExtraTileSetProperties()
             , prependRoot(fs::path(), resource(), root)));

    mc.position.orientation = { 0.0, -90.0, 0.0 };
    mc.position.verticalExtent = 1000;
    mc.position.verticalFov = 90;

    return mc;
}

Generator::Task SurfaceSpheroid
::generateFile_impl(const FileInfo &fileInfo, const Sink::pointer &sink) const
{
    SurfaceFileInfo fi(fileInfo, config().fileFlags);

    switch (fi.type) {
    case SurfaceFileInfo::Type::unknown:
        sink->error(utility::makeError<NotFound>("Unrecognized filename."));
        break;

    case SurfaceFileInfo::Type::file: {
        switch (fi.fileType) {
        case vts::File::config: {
            if (fi.raw) {
                sink->content(vs::fileIStream
                              (fi.fileType, filePath(vts::File::config)));
            } else {
                std::ostringstream os;
                mapConfig(os, ResourceRoot::none);
                sink->content(os.str(), fi.sinkFileInfo());
            }
            break;
        }
        case vts::File::tileIndex:
            sink->content(vs::fileIStream
                          (fi.fileType, filePath(vts::File::tileIndex)));
            break;

        default:
            sink->error(utility::makeError<InternalError>
                        ("Unsupported file"));
            break;
        }
        break;
    }

    case SurfaceFileInfo::Type::tile: {
        switch (fi.tileType) {
        case vts::TileFile::meta:
            return[=](GdalWarper &warper)  {
                generateMetatile(fi.tileId, sink, fi, warper);
            };

        case vts::TileFile::mesh:
            return[=](GdalWarper &warper)  {
                generateMesh(fi.tileId, sink, fi, warper);
            };

        case vts::TileFile::atlas:
            sink->error(utility::makeError<NotFound>
                        ("No internal texture present."));
            break;

        case vts::TileFile::navtile:
            return[=](GdalWarper &warper)  {
                generateNavtile(fi.tileId, sink, fi, warper);
            };
            break;
        }
        break;
    }

    case SurfaceFileInfo::Type::support:
        sink->content(fi.support->data, fi.support->size
                      , fi.sinkFileInfo(), false);
        break;

    default:
        sink->error(utility::makeError<InternalError>
                    ("Not implemented yet."));
    }

    return {};
}

namespace {

inline cv::Vec3d asVec(const math::Point3 &p)
{
    return { p(0), p(1), p(2) };
}

inline math::Point3 asPoint(const cv::Vec3d &v)
{
    return { v(0), v(1), v(2) };
}

inline const math::Point3& asPoint(const math::Point3 &p)
{
    return p;
}

typedef vts::MetaNode::Flag MetaFlag;
typedef vts::TileIndex::Flag TiFlag;

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

inline geo::SrsDefinition setGeoid(const std::string &srs
                                   , const std::string &geoidGrid)
{
    return geo::setGeoid(vr::Registry::srs(srs).srsDef, geoidGrid);
}

vts::CsConvertor sds2phys(const std::string &sds
                          , const vr::ReferenceFrame &rf
                          , const boost::optional<std::string> &geoidGrid)
{
    if (!geoidGrid) {
        return vts::CsConvertor(sds, rf.model.physicalSrs);
    }

    // force given geoid
    return vts::CsConvertor
        (setGeoid(sds, *geoidGrid), rf.model.physicalSrs);
}

vts::CsConvertor sds2nav(const std::string &sds
                         , const vr::ReferenceFrame &rf
                         , const boost::optional<std::string> &geoidGrid)
{
    if (!geoidGrid) {
        return vts::CsConvertor(sds, rf.model.navigationSrs);
    }

    // force given geoid
    return vts::CsConvertor
        (setGeoid(sds, *geoidGrid), rf.model.navigationSrs);
}

const int metatileSamplesPerTile(8);

template <typename T>
class Grid {
public:
    typedef T value_type;

    Grid(const math::Size2 &size, const T &fill = T())
        : size_(size)
        , grid_(math::area(size), fill)
    {}

    T& operator()(int x, int y) {
        return grid_[index(x, y)];
    }

    const T& operator()(int x, int y) const {
        return grid_[index(x, y)];
    }

    /** Returns pointer to pixel of non if pixel is invalid
     */
    template <typename Mask>
    const T* operator()(const Mask &mask, int x, int y) const {
        if (!mask(x, y)) { return nullptr; }
        return &grid_[index(x, y)];
    }

private:
    inline int index(int x, int y) const {
#ifndef NDEBUG
        // this is compiled in only in debug mode
        if ((x < 0) || (x >= size_.width)
            || (y < 0) || (y >= size_.height))
        {
            LOGTHROW(err3, Error)
                << "Invalid index [" << x << ", " << y << "] in grid of size "
                << size_ << ". Go and fix your code,";
        }
#endif // NDEBUG
        return y * size_.width + x;
    }

    math::Size2 size_;
    std::vector<T> grid_;
};

class ShiftMask {
public:
    ShiftMask(const MetatileBlock &block, int samplesPerTile)
        : offset_(block.offset)
        , size_((1 << offset_.lod) * samplesPerTile + 1
                , (1 << offset_.lod) * samplesPerTile + 1)
        , mask_(block.commonAncestor.coverageMask
                (vts::NodeInfo::CoverageType::grid, size_, 1))
    {}

    bool operator()(int x, int y) const {
        return mask_.get(x + offset_.x, y + offset_.y);
    }

private:
    const vts::TileId offset_;
    const math::Size2 size_;
    const vts::NodeInfo::CoverageMask mask_;
};

/** Calculates area of a quad (tries both diagonals)
 *
 *  Returns computed area and number of triangles that make up the area
 */
std::tuple<double, int> quadArea(const math::Point3 *v00
                                 , const math::Point3 *v01
                                 , const math::Point3 *v10
                                 , const math::Point3 *v11)
{
    std::tuple<double, int> qa;

    // lower
    if (v10 && v00) {
        // try both diagonals
        if (v11) {
            std::get<0>(qa) += vts::triangleArea(*v10, *v11, *v00);
            ++std::get<1>(qa);
        } else if (v01) {
            std::get<0>(qa) += vts::triangleArea(*v00, *v10, *v01);
            ++std::get<1>(qa);
        }
    }

    // upper
    if (v11 && v01) {
        // try both diagonals
        if (v00) {
            std::get<0>(qa) += vts::triangleArea(*v11, *v01, *v00);
            ++std::get<1>(qa);
        } else if (v10) {
            std::get<0>(qa) += vts::triangleArea(*v10, *v11, *v01);
            ++std::get<1>(qa);
        }
    }

    return qa;
}

} // namespace

void SurfaceSpheroid::generateMetatile(const vts::TileId &tileId
                                       , const Sink::pointer &sink
                                       , const SurfaceFileInfo &fi
                                       , GdalWarper&) const
{
    sink->checkAborted();

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
        const auto &extents(block.extents);
        const auto es(math::size(extents));
        const math::Size2 bSize(vts::tileRangesSize(view));

        const math::Size2 gridSize
            (bSize.width * metatileSamplesPerTile + 1
             , bSize.height * metatileSamplesPerTile + 1);

        LOG(info1) << "Processing metatile block ["
                   << vts::tileId(tileId.lod, block.view.ll)
                   << ", " << vts::tileId(tileId.lod, block.view.ur)
                   << "], ancestor: " << block.commonAncestor.nodeId()
                   << ", tile offset: " << block.offset;

        // grid (in grid coordinates); fill in with invalid numbers
        Grid<math::Point3> grid
            (gridSize, math::Point3(std::numeric_limits<double>::quiet_NaN()));

        // grid mask
        const ShiftMask mask(block, metatileSamplesPerTile);

        // tile size in grid and in real SDS
        math::Size2f gts
            (es.width / (metatileSamplesPerTile * bSize.width)
             , es.height / (metatileSamplesPerTile * bSize.height));
        math::Size2f ts(es.width / bSize.width
                        , es.height / bSize.height);

        auto conv(sds2phys(block.srs, rf, definition_.geoidGrid));
        auto navConv(sds2nav(block.srs, rf, definition_.geoidGrid));

        // fill in matrix
        for (int j(0), je(gridSize.height); j < je; ++j) {
            auto y(extents.ur(1) - j * gts.height);
            for (int i(0), ie(gridSize.width); i < ie; ++i) {
                // work only with non-masked pixels
                if (mask(i, j)) {
                    grid(i, j)
                        = conv(math::Point3
                               (extents.ll(0) + i * gts.width, y, 0.0));
                }
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

                for (int jj(0); jj <= metatileSamplesPerTile; ++jj) {
                    auto yy(j * metatileSamplesPerTile + jj);
                    for (int ii(0); ii <= metatileSamplesPerTile; ++ii) {
                        auto xx(i * metatileSamplesPerTile + ii);
                        const auto *p(grid(mask, xx, yy));

                        // update tile extents (if point valid)
                        if (p) { math::update(te, *p); }

                        if (geometry && ii && jj) {
                            // compute area of the quad composed of 1 or 2
                            // triangles
                            auto qa(quadArea(grid(mask, xx - 1, yy - 1)
                                             , p
                                             , grid(mask, xx - 1, yy)
                                             , grid(mask, xx, yy - 1)));
                            area += std::get<0>(qa);
                            triangleCount += std::get<1>(qa);
                        }

                        if (p && navtile) {
                            // sample height in navtile
                            auto z(navConv
                                   (math::Point3
                                    (extents.ll(0) + xx * gts.width
                                     , yy, 0.0))(2));
                            update(heightRange, z);
                        }
                    }
                }

                if (!area) {
                    // well, empty tile, no children
                    continue;
                }

                // set extents
                node.extents = vr::normalizedExtents(rf, te);

                // build height range
                node.heightRange.min = std::floor(heightRange.min);
                node.heightRange.max = std::ceil(heightRange.max);

                // set credits
                node.updateCredits(resource().credits);

                // mesh is (almost) flat -> use tile area
                if (geometry) {
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
                    // LOG(info4)
                    //     << std::fixed << nodeId << " meshArea: " << area;
                    // LOG(info4)
                    //     << nodeId << " textureArea: " << textureArea;

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

namespace {

class TextureNormalizer {
public:
    TextureNormalizer(const math::Extents2 &divisionExtents)
        : size_(size(divisionExtents))
        , origin_(divisionExtents.ll - center(divisionExtents))
    {}

    math::Point2 operator()(const math::Point3 &p) const {
        return math::Point2((p(0) - origin_(0)) / size_.width
                            , (p(1) - origin_(1)) / size_.height);
    };

private:
    math::Size2f size_;
    math::Point2 origin_;
};

void meshCoverageMask(vts::Mesh::CoverageMask &mask, const geometry::Mesh &mesh
                      , const math::Extents2 &extents, bool fullyCovered)
{
    const auto size(vts::Mesh::coverageSize());
    if (fullyCovered) {
        mask = vts::Mesh::CoverageMask(size, vts::Mesh::CoverageMask::FULL);
        return;
    }

    // mesh trafo
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));
    {

        const auto gridSize(vts::Mesh::coverageSize());
        const auto es(math::size(extents));
        math::Size2f scale(gridSize.width / es.width
                           , gridSize.height / es.height);

        // scale (NB: vertical flip)
        trafo(0, 0) = scale.width;
        trafo(1, 1) = -scale.height;

        // shift (by half grid and also by half pixel to move pixel centers at
        // integer indices)
        trafo(0, 3) = gridSize.width / 2.0 - 0.5;
        trafo(1, 3) = gridSize.height / 2.0 - 0.5;
    }

    mask = vts::Mesh::CoverageMask(size, vts::Mesh::CoverageMask::EMPTY);
    std::vector<imgproc::Scanline> scanlines;
    cv::Point3f tri[3];
    for (const auto &face : mesh.faces) {
        int i(0);
        for (auto v : { face.a, face.b, face.c }) {
            auto p(transform(trafo, mesh.vertices[v]));
            tri[i].x = p(0);
            tri[i].y = p(1);
            tri[i].z = p(2);
            ++i;
        }

        scanlines.clear();
        imgproc::scanConvertTriangle(tri, 0, size.height, scanlines);

        for (const auto &sl : scanlines) {
            imgproc::processScanline(sl, 0, size.width
                                     , [&](int x, int y, float)
            {
                mask.set(x, y);
            });
        }
    }
}

} // namespace

void SurfaceSpheroid::generateMesh(const vts::TileId &tileId
                                   , const Sink::pointer &sink
                                   , const SurfaceFileInfo &fi
                                   , GdalWarper&) const
{
    // TODO: calculate tile sampling
    const int samplesPerSide(128);
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

    const auto extents(nodeInfo.extents());
    const auto ts(math::size(nodeInfo.extents()));

    // generate mesh
    auto meshWithCoverage
        (meshFromNode(nodeInfo, math::Size2(samplesPerSide, samplesPerSide)));
    auto &lm(std::get<0>(meshWithCoverage));

    // simplify
    simplifyMesh(lm, nodeInfo, facesPerTile);

    // fake texture coordinate, needed by skirt
    lm.tCoords.emplace_back();

    // skirt (use just tile-size width)
    lm.skirt(math::Point3(0.0, 0.0, - 0.01 * ts.width));

    vts::Mesh mesh;

    // create submesh from local mesh
    auto l2g(geo::local2geo(extents));

    TextureNormalizer tn(extents);
    mesh.submeshes.emplace_back();
    auto &sm(mesh.submeshes.back());

    sm.textureMode = vts::SubMesh::external;
    if (definition_.textureLayerId) {
        sm.textureLayer = definition_.textureLayerId;
    }

    bool generateEtc(nodeInfo.node().externalTexture);
    auto conv(sds2phys(nodeInfo.srs(), rf, definition_.geoidGrid));
    for (const auto &v : lm.vertices) {
        // convert v from local coordinates to division SRS then to physical SRS
        sm.vertices.push_back(conv(transform(l2g, v)));

        // generate external texture coordinates if instructed
        if (generateEtc) {
            sm.etc.push_back(tn(v));
        }
    }

    for (const auto &f : lm.faces) {
        sm.faces.emplace_back(f.a, f.b, f.c);
    }

    if (fi.raw) {
        // generate coverage mask
        meshCoverageMask(mesh.coverageMask, lm, extents
                         , std::get<1>(meshWithCoverage).full());
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

void SurfaceSpheroid::generateNavtile(const vts::TileId &tileId
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
    auto navConv(sds2nav(nodeInfo.srs(), rf, definition_.geoidGrid));

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
