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

#ifndef mapproxy_calipers_calipers_hpp_included_
#define mapproxy_calipers_calipers_hpp_included_

#include <boost/optional.hpp>

#include "utility/enum-io.hpp"

#include "geo/geodataset.hpp"

#include "vts-libs/registry.hpp"
#include "vts-libs/vts/basetypes.hpp"

namespace calipers {

UTILITY_GENERATE_ENUM_CI(DatasetType,
                         ((dem)("dem")("dsm"))
                         ((ophoto))
                         )

struct Config {
    boost::optional<DatasetType> datasetType;
    double demToOphotoScale;
    double tileFractionLimit;

    Config()
        : demToOphotoScale(3.0), tileFractionLimit(32.0)
    {}
};

/** Calipers measurement
 */
struct Measurement {
    struct Node {
        std::string srs;
        vtslibs::vts::Ranges ranges;

        Node() = default;
        typedef std::vector<Node> list;
    };

    DatasetType datasetType;
    double gsd;
    vtslibs::vts::LodRange lodRange;
    vtslibs::vts::TileRange tileRange;
    Node::list nodes;
    vtslibs::registry::Position position;

    Measurement()
        : gsd()
        , lodRange(vtslibs::vts::LodRange::emptyRange())
        , tileRange(math::InvalidExtents{})
    {}

    vtslibs::vts::LodTileRange::list lodTileRanges() const;
};

Measurement measure(const vtslibs::registry::ReferenceFrame &referenceFrame
                    , const geo::GeoDataset::Descriptor &dataset
                    , const Config &config);

} // namespace calipers

#endif // mapproxy_calipers_calipers_hpp_included_
