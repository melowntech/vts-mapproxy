#include "math/math.hpp"
#include "geometry/meshop.hpp"
#include "imgproc/scanconversion.hpp"
#include "geo/coordinates.hpp"

#include "vts-libs/vts/math.hpp"
#include "vts-libs/vts/csconvertor.hpp"

#include "./mesh.hpp"
#include "./srs.hpp"

namespace vr = vadstena::registry;

std::tuple<geometry::Mesh, bool>
meshFromNode(const vts::NodeInfo &nodeInfo, const math::Size2 &edges
             , const HeightSampler &heights)
{
    std::tuple<geometry::Mesh, bool> res;
    auto &fullyCovered(std::get<1>(res) = false);

    const auto extents(nodeInfo.extents());
    const auto ts(math::size(extents));

    // one tile pixel
    math::Size2f px(ts.width / edges.width, ts.height / edges.height);

    cv::Mat_<int> indices(edges.width + 1, edges.height + 1, -1);

    auto g2l(geo::geo2local(extents));

    // get node coverage
    auto coverage(nodeInfo.coverageMask
                  (vts::NodeInfo::CoverageType::grid
                   , math::Size2(edges.width + 1, edges.height + 1)
                   , 1));

    if (coverage.empty()) { return res; }
    fullyCovered = coverage.full();

    // fill grid and remember indices
    auto &lm(std::get<0>(res));
    for (int j(0), je(edges.height); j <= je; ++j) {
        auto y(extents.ur(1) - j * px.height);
        for (int i(0), ie(edges.width); i <= ie; ++i) {
            if (!coverage.get(i, j)) { continue; }

            // sample height
            double height(0.0);
            if (heights && !heights(i, j, height)) {
                fullyCovered = false;
                continue;
            }

            // remember vertex index
            indices(j, i) = lm.vertices.size();

            // create vertex in SDS (local to extents center)
            lm.vertices.push_back
                (math::transform
                 (g2l, math::Point3(extents.ll(0) + i * px.width, y
                                    , height)));
        }
    }

    auto getVertex([&](int i, int j) -> const int*
    {
        const auto &index(indices(j, i));
        if (index < 0) { return nullptr; }
        return &index;
    });

    // mesh the grid
    for (int j(0), je(edges.height); j < je; ++j) {
        for (int i(0), ie(edges.width); i < ie; ++i) {
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

    return res;
}

namespace {

/** Computes area of 3D mesh and area of its projecteion to X-Y plane.
 *  Projection is determined as an area of triangle with Z cooridnated set to
 *  zero. Therefore, mesh cannot have overlapping trinagles.
 */
std::pair<double, double> meshArea(const geometry::Mesh &mesh)
{
    std::pair<double, double> res(0.0, 0.0);
    auto &area(res.first);
    auto &projected(res.second);

    for (const auto &face : mesh.faces) {
        auto a(mesh.vertices[face.a]);
        auto b(mesh.vertices[face.b]);
        auto c(mesh.vertices[face.c]);

        // area in 3D
        area += vts::triangleArea(a, b, c);

        // reset z coordinate -> area in 2D
        a[2] = b[2] = c[2] = 0.0;
        projected += vts::triangleArea(a, b, c);
    }

    return res;
}

double meshArea(const geometry::Mesh &mesh, const math::Points3d &alt)
{
    double area(0.0);

    for (const auto &face : mesh.faces) {
        auto a(alt[face.a]);
        auto b(alt[face.b]);
        auto c(alt[face.c]);

        // area in 3D
        area += vts::triangleArea(a, b, c);
    }

    return area;
}

inline math::Matrix4 geo2normalized(const math::Extents3 &extents)
{
    const math::Point3 midpoint(math::center(extents));

    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));
    trafo(0, 0) = (2.0 / (extents.ur[0] - extents.ll[0]));
    trafo(1, 1) = (2.0 / (extents.ur[1] - extents.ll[1]));
    trafo(2, 2) = (2.0 / (extents.ur[2] - extents.ll[2]));
    trafo(0, 3) = -midpoint[0] * trafo(0, 0);
    trafo(1, 3) = -midpoint[1] * trafo(1, 1);
    trafo(2, 3) = -midpoint[2] * trafo(2, 2);

    return trafo;
}

} // namespace

void simplifyMesh(geometry::Mesh &mesh, const vts::NodeInfo &nodeInfo
                  , const TileFacesCalculator &tileFacesCalculator
                  , const boost::optional<std::string> &geoidGrid)
{
    const auto ts(math::size(nodeInfo.extents()));

    // calculate number of faces
    auto area(meshArea(mesh));
    const int faceCount(tileFacesCalculator(area.first, area.second));
    LOG(info1)
        << "Simplifying mesh to " << faceCount << " faces per tile (from "
        << mesh.faces.size() << ".";

    // simplify with locked inner border

    if (int(faceCount) < int(mesh.faces.size())) {
        // convert all vertives to physical space
        math::Points3d physv;
        {
            const auto extents(nodeInfo.extents());
            const auto l2g(geo::local2geo(extents));
            const auto conv(sds2phys(nodeInfo, geoidGrid));
            const auto g2n
                (geo2normalized(nodeInfo.referenceFrame().division.extents));
            for (const auto &v : mesh.vertices) {
                physv.push_back(transform(g2n, conv(transform(l2g, v))));
            }
        }

        // max edge is radius of tile divided by edges per side computed from
        // faces-per-tile
        const auto maxEdgeLength
            (std::sqrt((2 * area.second) / (faceCount / 2.0)));

        mesh = *simplify(mesh, faceCount
                         , geometry::SimplifyOptions
                         (geometry::SimplifyOption::INNERBORDER
                          | geometry::SimplifyOption::CORNERS
                          | geometry::SimplifyOption::PREVENTFACEFLIP)
                         .minAspectRatio(5)
                         .maxEdgeLength(maxEdgeLength)
                         .alternativeVertices(&physv)
                         );

        LOG(info1)
            << "Simplified mesh to " << mesh.faces.size()
            << " faces (should be " << faceCount
            << ", difference: " << (int(mesh.faces.size()) - int(faceCount))
            << ").";
    } else {
        LOG(info1) << "No need to simplify mesh.";
    }
}

void meshCoverageMask(vts::Mesh::CoverageMask &mask, const geometry::Mesh &mesh
                      , const vts::NodeInfo &nodeInfo, bool fullyCovered)
{
    const auto size(vts::Mesh::coverageSize());
    if (fullyCovered) {
        mask.recreate(vts::Mesh::coverageOrder, 1);
        return;
    }

    // mesh trafo
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));
    {

        const auto gridSize(vts::Mesh::coverageSize());
        const auto es(math::size(nodeInfo.extents()));
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

    mask.recreate(vts::Mesh::coverageOrder, 0);
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
                mask.set(x, y, 1);
            });
        }
    }
}

void addSkirt(geometry::Mesh &mesh, const vts::NodeInfo &nodeInfo)
{
    const auto skirtHeight(math::size(nodeInfo.extents()).width * 0.01);

    // fake texture coordinate, needed by skirt
    mesh.tCoords.emplace_back();

    // skirt (use just tile-size width)
    mesh.skirt(math::Point3(0.0, 0.0, -skirtHeight));
}

vts::CsConvertor sds2srs(const std::string &sds, const std::string &dst
                         , const boost::optional<std::string> &geoidGrid)
{
    if (!geoidGrid) {
        return vts::CsConvertor(sds, dst);
    }

    // force given geoid
    return vts::CsConvertor
        (geo::setGeoid(vr::system.srs(sds).srsDef, *geoidGrid)
         , dst);
}

vts::CsConvertor sds2phys(const vts::NodeInfo &nodeInfo
                          , const boost::optional<std::string> &geoidGrid)
{
    return sds2srs(nodeInfo.srs()
                   , nodeInfo.referenceFrame().model.physicalSrs
                   , geoidGrid);
}

vts::CsConvertor sds2nav(const vts::NodeInfo &nodeInfo
                         , const boost::optional<std::string> &geoidGrid)
{
    return sds2srs(nodeInfo.srs()
                   , nodeInfo.referenceFrame().model.navigationSrs
                   , geoidGrid);
}

geo::SrsDefinition sds(const vts::NodeInfo &nodeInfo
                       , const boost::optional<std::string> &geoidGrid)
{
    if (!geoidGrid) { return nodeInfo.srsDef(); }

    // force given geoid
    return geo::setGeoid(nodeInfo.srsDef(), *geoidGrid);
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

} // namespace

vts::SubMesh& addSubMesh(vts::Mesh &mesh, const geometry::Mesh &gmesh
                         , const vts::NodeInfo &nodeInfo
                         , const boost::optional<std::string> &geoidGrid)
{
    const auto extents(nodeInfo.extents());
    const auto l2g(geo::local2geo(extents));

    mesh.submeshes.emplace_back();
    auto &sm(mesh.submeshes.back());
    sm.textureMode = vts::SubMesh::external;

    TextureNormalizer tn(extents);
    const auto conv(sds2phys(nodeInfo, geoidGrid));

    bool generateEtc(nodeInfo.node().externalTexture);
    for (const auto &v : gmesh.vertices) {
        // convert v from local coordinates to division SRS then to physical SRS
        sm.vertices.push_back(conv(transform(l2g, v)));

        // generate external texture coordinates if instructed
        if (generateEtc) {
            sm.etc.push_back(tn(v));
        }
    }

    for (const auto &f : gmesh.faces) {
        sm.faces.emplace_back(f.a, f.b, f.c);
    }

    return sm;
}

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

int TileFacesCalculator::operator()(double meshArea, double meshProjectedArea)
    const
{
    // ratio between mesh area and area of its projection to tile base
    const auto areaRatio(meshArea / meshProjectedArea);

    // calculate scaling factor
    auto factor((std::pow(quotient_, areaRatio) - 1.0)
                      / (quotient_ - 1.0));

    // clamp factor to min/max range
    factor = math::clamp(factor, roughnessFactorMin_, roughnessFactorMax_);

    // apply factor to base number of faces
    return base_* factor;
}
