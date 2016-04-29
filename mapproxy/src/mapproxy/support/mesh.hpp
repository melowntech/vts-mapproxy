#ifndef mapproxy_support_mesh_hpp_included_
#define mapproxy_support_mesh_hpp_included_

#include <tuple>

#include <boost/optional.hpp>

#include "geometry/mesh.hpp"

#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/mesh.hpp"

namespace vts = vadstena::vts;

std::tuple<geometry::Mesh, bool>
meshFromNode(const vts::NodeInfo &nodeInfo, const math::Size2 &edges);

void simplifyMesh(geometry::Mesh &mesh, const vts::NodeInfo &nodeInfo
                  , int facesPerTile);

void meshCoverageMask(vts::Mesh::CoverageMask &mask, const geometry::Mesh &mesh
                      , const vts::NodeInfo &nodeInfo, bool fullyCovered);

void addSkirt(geometry::Mesh &mesh, const vts::NodeInfo &nodeInfo);

vts::SubMesh& addSubMesh(vts::Mesh &mesh, const geometry::Mesh &gmesh
                         , const vts::NodeInfo &nodeInfo
                         , const boost::optional<std::string> &geoidGrid);

#endif // mapproxy_support_mesh_hpp_included_
