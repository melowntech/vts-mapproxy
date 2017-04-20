/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef mapproxy_support_mesh_hpp_included_
#define mapproxy_support_mesh_hpp_included_

#include <tuple>

#include <boost/optional.hpp>

#include "geometry/mesh.hpp"

#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/mesh.hpp"

namespace vts = vtslibs::vts;

/** Calculates number of faces to simplify mesh to based on ideal flat tile and
 *  tile's roughness.
 */
class TileFacesCalculator {
public:
    TileFacesCalculator()
        : base_(1000), roughnessFactorMin_(0.0), roughnessFactorMax_(3.0)
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
                  , const TileFacesCalculator &tileFacesCalculator
                  , const boost::optional<std::string> &geoidGrid);

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
