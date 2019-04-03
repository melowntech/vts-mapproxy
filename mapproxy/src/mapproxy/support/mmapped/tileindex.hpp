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

#ifndef mapproxy_support_mmapped_tileindex_hpp_included_
#define mapproxy_support_mmapped_tileindex_hpp_included_

#include <array>
#include <iostream>

#include "vts-libs/vts/tileindex.hpp"
#include "vts-libs/vts/tileset/tilesetindex.hpp"

#include "memory.hpp"
#include "tileflags.hpp"
#include "qtree.hpp"

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

class TileIndex {
public:
    typedef TileFlag::value_type value_type;

    TileIndex(const boost::filesystem::path &path);

    /** Find tile value.
     */
    value_type get(const vts::TileId &tileId) const;

    bool validSubtree(vts::Lod lod, const vts::TileId &tileId) const;

    bool validSubtree(const vts::TileId &tileId) const;

    /** Get quad tree for given lod.
     */
    const QTree* tree(vts::Lod lod) const;

    value_type checkMask(const vts::TileId &tileId, QTree::value_type mask)
        const;

    /** Save vts TileIndex into this mmapped tile index.
     */
    static void write(std::ostream &out, const vts::TileIndex &ti);

    /** Save vts TileIndex into this mmapped tile index.
     */
    static void write(const boost::filesystem::path &path
                      , const vts::TileIndex &ti);

private:
    std::shared_ptr<Memory> memory_;
    QTree::list trees_;
};

// inlines

inline TileIndex::value_type
TileIndex::checkMask(const vts::TileId &tileId, QTree::value_type mask) const
{
    return get(tileId) & mask;
}


} // namespace mmapped

#endif // mapproxy_support_mmapped_tileindex_hpp_included_
