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

#include "tilesetindex.hpp"

namespace mmapped {

bool Index::meta(const vts::TileId &tileId) const
{
    return tileIndex.validSubtree
        (tileId.lod, vts::parent(tileId, metaBinaryOrder_));
}

bool Index::check(const vts::TileId &tileId, vts::TileFile type) const
{
    switch (type) {
    case vts::TileFile::meta:
        return meta(tileId);
    case vts::TileFile::mesh:
        return tileIndex.checkMask(tileId, vts::TileIndex::Flag::mesh);
    case vts::TileFile::atlas:
        return tileIndex.checkMask(tileId, vts::TileIndex::Flag::atlas);
    case vts::TileFile::navtile:
        return tileIndex.checkMask(tileId, vts::TileIndex::Flag::navtile);

    default: return false;
    }
}

vts::TileIndex::Flag::value_type
Index::checkAndGetFlags(const vts::TileId &tileId, vts::TileFile type) const
{
    const auto flags(tileIndex.get(tileId));

    switch (type) {
    case vts::TileFile::mesh:
        return (flags & vts::TileIndex::Flag::mesh) ? flags : 0;
    case vts::TileFile::atlas:
        return (flags & vts::TileIndex::Flag::atlas) ? flags : 0;
    case vts::TileFile::navtile:
        return (flags & vts::TileIndex::Flag::navtile) ? flags : 0;

    default: return 0;
    }
}

} // namespace mmapped
