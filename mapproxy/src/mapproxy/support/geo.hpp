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

#ifndef mapproxy_support_geo_hpp_included_
#define mapproxy_support_geo_hpp_included_

#include "geo/geodataset.hpp"

#include "vts-libs/vts/nodeinfo.hpp"

namespace vts = vtslibs::vts;
namespace vr = vtslibs::registry;

// fwd
class OGRSpatialReference;

struct DemDataset {
    std::string dataset;
    boost::optional<std::string> geoidGrid;

    DemDataset() = default;

    DemDataset(const std::string &dataset
               , const boost::optional<std::string> &geoidGrid = boost::none)
        : dataset(dataset), geoidGrid(geoidGrid)
    {}

    bool operator==(const DemDataset &other) const;
    bool operator!=(const DemDataset &other) const;

    typedef std::vector<DemDataset> list;
};

double tileCircumference(const math::Extents2 &extents
                         , const geo::SrsDefinition &srs
                         , const geo::GeoDataset &dataset
                         , int samples = 20);

math::Extents2 extentsPlusHalfPixel(const math::Extents2 &extents
                                    , const math::Size2 &pixels);

/** Local Transversal Mercator.
 */
::OGRSpatialReference localTm(const vr::ReferenceFrame &rf
                              , const geo::SrsDefinition &srsDef
                              , const math::Point2d point);

/** Constructs trasformation matrix for projecting physical coordinates to plane
 *  defined by point in navigation coordinates.
 */
math::Matrix4 makePlaneTrafo(const vr::ReferenceFrame &rf
                             , const math::Point2 &navCenter);

// inlines

inline bool DemDataset::operator==(const DemDataset &other) const
{
    return ((dataset == other.dataset)
            && (geoidGrid == other.geoidGrid));
}

inline bool DemDataset::operator!=(const DemDataset &other) const
{
    return !operator==(other);
}

#endif // mapproxy_support_geo_hpp_included_
