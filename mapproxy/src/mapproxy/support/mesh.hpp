#ifndef mapproxy_support_mesh_hpp_included_
#define mapproxy_support_mesh_hpp_included_

#include <tuple>

#include <boost/optional.hpp>

#include "geometry/mesh.hpp"

#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/mesh.hpp"

namespace vts = vadstena::vts;

/** Calculates number of faces to simplify mesh to based on ideal flat tile and
 *  tile's roughness.
 */
class TileFacesCalculator {
public:
    TileFacesCalculator()
        : base_(1500), roughnessFactorMin_(0.0), roughnessFactorMax_(3.0)
        , quotient_(1.0 - (1.0 / roughnessFactorMax_))
    {}

    int operator()(double meshArea, double meshProjectedArea) const;

private:
    int base_;
    double roughnessFactorMin_;
    double roughnessFactorMax_;
    double quotient_;
};

typedef std::function<bool(int, int, double&)> HeightSampler;

std::tuple<geometry::Mesh, bool>
meshFromNode(const vts::NodeInfo &nodeInfo, const math::Size2 &edges
             , const HeightSampler &heights = HeightSampler());

void simplifyMesh(geometry::Mesh &mesh, const vts::NodeInfo &nodeInfo
                  , const TileFacesCalculator &tileFacesCalculator);

void meshCoverageMask(vts::Mesh::CoverageMask &mask, const geometry::Mesh &mesh
                      , const vts::NodeInfo &nodeInfo, bool fullyCovered);

void addSkirt(geometry::Mesh &mesh, const vts::NodeInfo &nodeInfo);

vts::SubMesh& addSubMesh(vts::Mesh &mesh, const geometry::Mesh &gmesh
                         , const vts::NodeInfo &nodeInfo
                         , const boost::optional<std::string> &geoidGrid);

/** Calculates area of a quad.
 *
 *  Returns computed area and number of triangles that make up the area.  Both
 *  diagonals are tried (if one configuration doesn't work the other is tried
 *  instead).
 *
 *  Returns area of a quad and number of valid triangles (one of { 0, 1, 2 })
 *
 *  Quad is constructed as:
 *         v00 ------ v01
 *          |          |
 *          |          |
 *          |          |
 *         v10 ------ v11
 *
 */
std::tuple<double, int> quadArea(const math::Point3 *v00
                                 , const math::Point3 *v01
                                 , const math::Point3 *v10
                                 , const math::Point3 *v11);

#endif // mapproxy_support_mesh_hpp_included_
