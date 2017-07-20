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

#ifndef mapproxy_support_mmapped_tileflags_hpp_included_
#define mapproxy_support_mmapped_tileflags_hpp_included_

#include <array>
#include <iostream>

#include "vts-libs/vts/tileindex.hpp"

namespace vts = vtslibs::vts;

/** Simplified tileindex that employs memory mapped quadtrees.
 *
 *  Used to save precious memory.
 *
 *  Handles only 3 flags: mesh, watertight and navtile flags, space for 5 more
 *  flags is available.
 *
 *  Non-leaf nodes are marked by invalid combination (mesh=false,
 *  watertight=true)
 */
namespace mmapped {

struct TileFlag {
    typedef std::uint8_t value_type;

    enum : value_type {
        mesh = vts::TileIndex::Flag::mesh // mesh (or some other data) present
        , watertight = vts::TileIndex::Flag::watertight // no holes in data
        , navtile = vts::TileIndex::Flag::navtile // navile present

        , data = mesh // alias for mesh
        , none = 0x00 // nothing set
        , any = 0xff // anything set
        /** invalid value used for internal tree nodes
         */
        , invalid = any & ~watertight
    };

    static bool leaf(value_type value) { return value != invalid; }
    static bool internal(value_type value) { return value == invalid; }
};

} // namespace mmapped

#endif // mapproxy_support_mmapped_tileflags_hpp_included_
