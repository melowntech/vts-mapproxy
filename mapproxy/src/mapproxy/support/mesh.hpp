#ifndef mapproxy_support_mesh_hpp_included_
#define mapproxy_support_mesh_hpp_included_

#include <tuple>

#include "geometry/mesh.hpp"

#include "vts-libs/vts/nodeinfo.hpp"

namespace vts = vadstena::vts;

std::tuple<geometry::Mesh, vts::NodeInfo::CoverageMask>
meshFromNode(const vts::NodeInfo &nodeInfo, const math::Size2 &edges);

void simplifyMesh(geometry::Mesh &mesh, const vts::NodeInfo &nodeInfo
                  , int facesPerTile);

#endif // mapproxy_support_mesh_hpp_included_
