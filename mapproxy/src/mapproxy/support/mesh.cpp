#include "math/math.hpp"

#include "geometry/meshop.hpp"

#include "geo/coordinates.hpp"

#include "vts-libs/vts/math.hpp"

#include "./mesh.hpp"

std::tuple<geometry::Mesh, vts::NodeInfo::CoverageMask>
meshFromNode(const vts::NodeInfo &nodeInfo, const math::Size2 &edges)
{
    std::tuple<geometry::Mesh, vts::NodeInfo::CoverageMask> res;

    const auto extents(nodeInfo.extents());
    const auto ts(math::size(extents));

    // one tile pixel
    math::Size2f px(ts.width / edges.width, ts.height / edges.height);

    cv::Mat_<int> indices(edges.width + 1, edges.height + 1
                          , -1);

    auto g2l(geo::geo2local(extents));

    // get node coverage
    auto &coverage(std::get<1>(res) = nodeInfo.coverageMask
                   (vts::NodeInfo::CoverageType::grid
                    , math::Size2(edges.width + 1, edges.height + 1)
                    , 1));

    if (coverage.empty()) { return res; }

    // fill grid and remember indices
    auto &lm(std::get<0>(res));
    for (int j(0), je(edges.height); j <= je; ++j) {
        auto y(extents.ur(1) - j * px.height);
        for (int i(0), ie(edges.width); i <= ie; ++i) {
            if (!coverage.get(i, j)) { continue; }

            // remember vertex index
            indices(j, i) = lm.vertices.size();

            // create vertex in SDS (local to extents center)
            lm.vertices.push_back
                (math::transform
                 (g2l, math::Point3(extents.ll(0) + i * px.width, y, 0.0)));
        }
    }

    auto getVertex([&](int i, int j) -> boost::optional<int>
    {
        auto index(indices(j, i));
        if (index < 0) { return boost::none; }
        return index;
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

} // namespace

void simplifyMesh(geometry::Mesh &mesh, const vts::NodeInfo &nodeInfo
                  , int facesPerTile)
{
    const auto ts(math::size(nodeInfo.extents()));

    // calculate number of faces
    const auto normalizedArea(calculateNormalizedMesh2dArea(mesh, ts));
    const int faceCount
        (std::round(normalizedArea * facesPerTile));
    LOG(info1)
        << "Simplifying mesh to " << faceCount << " faces per tile (from "
        << mesh.faces.size() << ".";

    // simplify with locked inner border
    {
        // max edge is radius of tile divided by edges per side computed from
        // faces-per-tile
        double maxEdgeLength
            (std::sqrt(math::sqr(ts.width) + math::sqr(ts.height))
             / std::sqrt(faceCount / 2.0));

        mesh = *simplify(mesh, faceCount
                         , geometry::SimplifyOptions
                         (geometry::SimplifyOption::INNERBORDER
                          | geometry::SimplifyOption::CORNERS
                          | geometry::SimplifyOption::PREVENTFACEFLIP)
                         .minAspectRatio(5)
                         .maxEdgeLength(maxEdgeLength));
    }

    LOG(info1)
        << "Simplified mesh to " << mesh.faces.size()
        << " faces (should be " << faceCount
        << ", difference: " << (int(mesh.faces.size()) - int(faceCount))
        << ").";
}

