#include <new>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"

#include "geometry/mesh.hpp"
#include "geometry/meshop.hpp"

#include "geo/coordinates.hpp"

#include "imgproc/rastermask/cvmat.hpp"
#include "imgproc/scanconversion.hpp"

#include "vts-libs/storage/fstreams.hpp"
#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/tileset/config.hpp"
#include "vts-libs/vts/metatile.hpp"
#include "vts-libs/vts/csconvertor.hpp"
#include "vts-libs/vts/math.hpp"
#include "vts-libs/vts/mesh.hpp"

#include "../error.hpp"
#include "../metatile.hpp"

#include "./surface-spheroid.hpp"
#include "./factory.hpp"

#include "browser2d/index.html.hpp"

namespace fs = boost::filesystem;
namespace vr = vadstena::registry;
namespace vs = vadstena::storage;

namespace cv {

/** OpenCV  traits for  math::Point3  data type.
 *
 * Simulates math::Point3 as N 64-bit channels where
 *     sizeof(math::Point3) <= channels * 8
 *
 * NB: matrix elemetns must be initialized by placement new!
 *
 * NB: there is no need * to call destructor because destructor of math::Points3
 *     is no-op
 */
template<> class DataType<math::Point3>
{
public:
    typedef math::Point3 value_type;
    typedef value_type work_type;
    typedef int channel_type;
    enum { generic_type = 0, depth = CV_64F // 8 bytes
           , channels = ((sizeof(math::Point3) + 7) / 8)
           , type = CV_MAKETYPE(depth, channels) };
    typedef Vec<channel_type, channels> vec_type;
};

} // namespace cv

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
            sink->error(utility::makeError<InternalError>
                        ("No navtile generated yet."));
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

MetaFlag::value_type ti2metaFlags(TiFlag::value_type ti)
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

::OGRSpatialReference setGeoid(const ::OGRSpatialReference &srs
                               , const std::string &geoid)
{
    if (!(srs.IsProjected() || srs.IsGeographic())) {
        LOGTHROW(err1, std::runtime_error)
            << "SRS set geoid: SRS is neither projected nor "
            "geographic coordinate system";
    }

    std::string wkt("VERT_CS[\"geoid height\","
                    "VERT_DATUM[\"geoid\",2005,EXTENSION[\"PROJ4_GRIDS\""
                    ",\"" + geoid + "\"]],UNIT[\"metre\",1]]");
    std::vector<char> tmp(wkt.c_str(), wkt.c_str() + wkt.size() + 1);
    ::OGRSpatialReference vert;
    char *data(tmp.data());
    auto err(vert.importFromWkt(&data));
    if (err != OGRERR_NONE) {
        LOGTHROW(err1, std::runtime_error)
            << "Error parsing wkt definition: <" << err << "> (input = "
            << wkt << ").";
    }

    OGRSpatialReference out;
    out.SetCompoundCS("", &srs, &vert);
    return out;
}

geo::SrsDefinition setGeoid(const std::string &srs
                            , const std::string &geoidGrid)
{
    auto def(vr::Registry::srs(srs).srsDef);
    auto out(geo::SrsDefinition::fromReference
             (setGeoid(def.reference(), geoidGrid)));
    LOG(info4) << "Setting geoid <" << geoidGrid << "> to <" << def << ">: "
               << out;
    return out;
}

vts::CsConvertor sds2phys(const std::string &sds
                          , const vr::ReferenceFrame &rf
                          , const boost::optional<std::string> &geoidGrid)
{
    if (!geoidGrid) {
        return vts::CsConvertor(sds, rf.model.physicalSrs);
    }

    // force given geoid
    return vts::CsConvertor(setGeoid(sds, *geoidGrid), rf.model.physicalSrs);
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

    int samplesPerTile(8);

    vts::MetaTile metatile(tileId, rf.metaBinaryOrder);

    for (const auto &block : blocks) {
        const auto &view(block.view);
        const auto &extents(block.extents);
        const auto es(math::size(extents));
        const math::Size2 bSize(vts::tileRangesSize(view));

        const math::Size2 gridSize(bSize.width * samplesPerTile + 1
                                   , bSize.height * samplesPerTile + 1);

        // grid (in grid coordinates)
        cv::Mat_<math::Point3> grid(gridSize.height, gridSize.width);

        // tile size in grid and in real SDS
        math::Size2f gts(es.width / (samplesPerTile * bSize.width)
                         , es.height / (samplesPerTile * bSize.height));
        math::Size2f ts(es.width / bSize.width
                        , es.height / bSize.height);

        auto conv(sds2phys(block.srs, rf, definition_.geoidGrid));

        // fill in matrix
        for (int j(0), je(gridSize.height); j < je; ++j) {
            auto y(extents.ur(1) - j * gts.height);
            for (int i(0), ie(gridSize.width); i < ie; ++i) {
                // convert point in SDS to world coordinates and store in the
                // grid
                // NB: we need to initialize the data -> placement new
                new (&grid(j, i)) math::Point3
                    (conv(math::Point3
                          (extents.ll(0) + i * gts.width, y, 0.0)));
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

                // compute tile extents
                math::Extents3 te(math::InvalidExtents{});
                double area(0);
                for (int jj(0); jj <= samplesPerTile; ++jj) {
                    auto yy(j * samplesPerTile + jj);
                    for (int ii(0); ii <= samplesPerTile; ++ii) {
                        auto xx(i * samplesPerTile + ii);
                        const auto &p(grid(yy, xx));

                        // update tile extents
                        math::update(te, p);

                        // TODO: use node mask!
                        if (geometry && ii && jj) {
                            // compute area of two triangles
                            const auto &p1(grid(yy - 1, xx - 1));
                            const auto &p2(grid(yy - 1, xx));
                            // p3 = p
                            const auto &p4(grid(yy, xx - 1));

                            area += vts::triangleArea(p1, p2, p4);
                            area += vts::triangleArea(p2, p, p4);
                        }
                    }
                }

                // set extents
                node.extents = vr::normalizedExtents(rf, te);
                // TODO: build height range

                // set credits
                node.updateCredits(resource().credits);

                // mesh is (almost) flat -> use tile area
                if (geometry) {
                    node.applyTexelSize(true);
                    // TODO: use node mask area ratio
                    node.texelSize = std::sqrt
                        (area / vr::BoundLayer::tileArea());
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

double calculateNormalizedMesh2dArea(const geometry::Mesh &mesh
                                     , const math::Size2f &tileSize)
{
    // mesh is in local coordinates, i.e. X and Y coordinates are in range
    // [-tileSize/2, tileSize/2]; to normalize them into range [-0.5, 0.5] we
    // have to divide them by tileSize

    double area(0.0);
    for (const auto &face : mesh.faces) {
        auto a(mesh.vertices[face.a]);
        auto b(mesh.vertices[face.b]);
        auto c(mesh.vertices[face.c]);
        // normalize
        a[0] /= tileSize.width; a[1] /= tileSize.height;
        b[0] /= tileSize.width; b[1] /= tileSize.height;
        c[0] /= tileSize.width; c[1] /= tileSize.height;
        // reset z coordinate
        a[2] = b[2] = c[2] = 0.0;
        area += vts::triangleArea(a, b, c);
    }

    // NB: area is normalized
    return area;
}

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

    const auto &extents(nodeInfo.extents());
    const auto ts(math::size(extents));

    // one tile pixel
    math::Size2f px(ts.width / samplesPerSide
                    , ts.height / samplesPerSide);

    cv::Mat_<int> indices(samplesPerSide + 1, samplesPerSide + 1
                          , -1);

    auto g2l(geo::geo2local(extents));

    // get node coverage
    auto coverage(nodeInfo.coverageMask
                  (vts::NodeInfo::CoverageType::grid
                   , math::Size2(samplesPerSide + 1, samplesPerSide + 1)
                   , 1));

    // fill grid and remember indices
    geometry::Mesh lm;
    for (int j(0), je(samplesPerSide); j <= je; ++j) {
        auto y(extents.ur(1) - j * px.height);
        for (int i(0), ie(samplesPerSide); i <= ie; ++i) {
            if (!coverage.get(i, j)) { continue; }

            // remember vertex index
            indices(j, i) = lm.vertices.size();

            // create vertex in SDS (local to extents center)
            lm.vertices.push_back
                (math::transform
                 (g2l, math::Point3(extents.ll(0) + i * px.width, y, 0.0)));
        }
    }

    if (lm.vertices.empty()) {
        sink->error(utility::makeError<NotFound>("No mesh for this tile."));
    }

    auto getVertex([&](int i, int j) -> boost::optional<int>
    {
        auto index(indices(j, i));
        if (index < 0) { return boost::none; }
        return index;
    });

    // mesh the grid
    for (int j(0), je(samplesPerSide); j < je; ++j) {
        for (int i(0), ie(samplesPerSide); i < ie; ++i) {
            auto v00(getVertex(i, j));
            auto v01(getVertex(i + 1, j));
            auto v10(getVertex(i, j + 1));
            auto v11(getVertex(i + 1, j + 1));

            // lower
            if (v10 && v00) {
                // try both diagonals
                if (v11) {
                    lm.addFace(*v10, *v11, *v00);
                } else if (v01) {
                    lm.addFace(*v00, *v10, *v01);
                }
            }

            // upper
            if (v11 && v01) {
                // try both diagonals
                if (v00) {
                    lm.addFace(*v11, *v01, *v00);
                } else if (v10) {
                    lm.addFace(*v10, *v11, *v01);
                }
            }
        }
    }

    // calculate number of faces
    const auto normalizedArea
        (calculateNormalizedMesh2dArea(lm, ts));
    const int faceCount
        (std::round(normalizedArea * facesPerTile));
    LOG(info1)
        << "Simplifying mesh to " << faceCount << " faces per tile (from "
        << lm.faces.size() << ".";

    // simplify with locked inner border
    {
        // max edge is radius of tile divided by edges per side computed from
        // faces-per-tile
        double maxEdgeLength
            (std::sqrt(math::sqr(ts.width) + math::sqr(ts.height))
             / std::sqrt(faceCount / 2.0));

        lm = *simplify(lm, faceCount
                       , geometry::SimplifyOptions
                       (geometry::SimplifyOption::INNERBORDER
                        | geometry::SimplifyOption::CORNERS
                        | geometry::SimplifyOption::PREVENTFACEFLIP)
                       .minAspectRatio(5)
                       .maxEdgeLength(maxEdgeLength));
    }

    // fake texture coordinate, needed by skirt
    lm.tCoords.emplace_back();

    LOG(info1)
        << "Simplified mesh to " << lm.faces.size()
        << " faces (should be " << faceCount
        << ", difference: " << (int(lm.faces.size()) - int(faceCount))
        << ").";

    // skirt (use just tile-size width)
    lm.skirt(math::Point3(0.0, 0.0, - 0.01 * ts.width));

    vts::Mesh mesh;

    // create submesh from local mesh
    auto l2g(geo::local2geo(extents));

    bool generateEtc(nodeInfo.node().externalTexture);
    TextureNormalizer tn(extents);
    mesh.submeshes.emplace_back();
    auto &sm(mesh.submeshes.back());

    sm.textureMode = vts::SubMesh::external;
    if (definition_.textureLayerId) {
        sm.textureLayer = definition_.textureLayerId;
    }

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
        meshCoverageMask(mesh.coverageMask, lm, extents, coverage.full());
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

} // namespace generator
