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

#ifndef mapproxy_support_mmtileindex_hpp_included_
#define mapproxy_support_mmtileindex_hpp_included_

#include "vts-libs/vts/tileindex.hpp"
#include "vts-libs/vts/tileset/tilesetindex.hpp"

namespace vts = vtslibs::vts;

/** Simplified tileindex that employs memory mapped quadtrees.
 *
 *  Used to save precious memory.
 *
 *  Handles only 3 flags: mesh, watertight and navtile flags, space for 4th flag
 *  is available.
 *
 *  Non-leaf nodes are marked by invalid combination (mesh=false,
 *  watertight=true)
 */
namespace mmapped {

/** Memory information.
 */
class Memory;

struct Flag {
    typedef std::uint8_t value_type;

    enum : value_type {
        mesh = 0x01 // mesh (or some other data) present
        , watertight = 0x02 // no holes in data
        , navtile = 0x04 // navile present

        , data = mesh // alias for mesh
        , none = 0x00 // nothing set
        , invalid = 0x0e // invalid value, used for internal tree nodes

        , rootMarker = 0xf0 // marks that root has single value
    };
};

class QTree {
public:
    typedef std::vector<QTree> list;

    /** Loads QTree from memory at current memory position.
     */
    QTree(Memory &memory);

    static void write(std::ostream &out, const vts::QTree &tree);

private:
    unsigned int depth_;
    const char *data_;
    std::size_t dataSize_;
};

class TileIndex {
public:
    TileIndex(const boost::filesystem::path &path);

    /** Save vts TileIndex into this mmapped tile index.
     */
    static void write(std::ostream &out, const vts::TileIndex &ti);

    static void write(const boost::filesystem::path &path
                      , const vts::TileIndex &ti);

private:
    std::shared_ptr<Memory> memory_;
    QTree::list trees_;
};

} // namespace mmapped


#endif // mapproxy_support_tileindex_hpp_included_
