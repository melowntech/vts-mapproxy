/**
 * Copyright (c) 2019 Melown Technologies SE
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

#ifndef mapproxy_support_tilejson_hpp_included_
#define mapproxy_support_tilejson_hpp_included_

#include <iostream>

#include <boost/optional.hpp>

#include "utility/enum-io.hpp"

#include "math/geometry_core.hpp"

#include "vts-libs/vts/basetypes.hpp"

namespace vts = vtslibs::vts;

/** Minimal needed TileJson file format implementation.
 */
struct LayerJson {
    typedef boost::optional<std::string> OString;
    enum class Scheme { xyz, tms };
    struct Version {
        int maj, min, patch;
        Version(int maj = 0, int min = 0, int patch = 0)
            : maj(maj), min(min), patch(patch)
        {}
    };

    typedef std::vector<vts::TileRange> TileRanges;
    typedef std::vector<TileRanges> Available;

    Version tilejson;
    Version version;
    OString format;
    OString name;
    OString description;
    Scheme scheme;
    OString projection;
    math::Extents2 bounds;
    vts::LodRange zoom;
    Available available;
    std::vector<std::string> tiles;
    OString attribution;

    LayerJson()
        : tilejson(2, 0, 0), version(1, 0, 0), scheme(Scheme::xyz)
        , bounds(-180, -90, 180, 90)
    {}
};

UTILITY_GENERATE_ENUM_IO(LayerJson::Scheme,
                         ((xyz))
                         ((tms)))

void save(const LayerJson &layer, std::ostream &os);

#endif // mapproxy_support_tilejson_hpp_included_
