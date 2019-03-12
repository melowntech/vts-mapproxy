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

#ifndef mapproxy_support_tms_hpp_included_
#define mapproxy_support_tms_hpp_included_

#include "vts-libs/vts/tileop.hpp"

namespace vts = vtslibs::vts;

vts::TileId tms2vts(const vts::TileId &rootId, bool flipY
                    , const vts::TileId &id);

vts::TileId vts2tms(const vts::TileId &rootId, bool flipY
                    , const vts::TileId &id);

vts::LodTileRange vts2tms(const vts::TileId &rootId, bool flipY
                          , const vts::LodTileRange &range);


// implementation

inline vts::TileId tms2vts(const vts::TileId &rootId, bool flipY
                           , const vts::TileId &id)
{
    // we can use vts::global function since it works this way (only sticking
    // tree as a subtree of given root)
    return vts::global
        (rootId, flipY ? vts::verticalFlip(id) : id);
}

inline vts::TileId vts2tms(const vts::TileId &rootId, bool flipY
                           , const vts::TileId &id)
{
    // well, we cannot use vts::local since TMS can have multi-tile root

    // sanity checks
    if (id.lod < rootId.lod) { return rootId; }

    const auto ldiff(id.lod - rootId.lod);
    vts::TileId tmsId(ldiff, id.x - (rootId.x << ldiff)
                      , id.y - (rootId.y << ldiff));
    return flipY ? vts::verticalFlip(tmsId) : tmsId;
}

inline vts::LodTileRange vts2tms(const vts::TileId &rootId, bool flipY
                                 , const vts::LodTileRange &range)
{
    auto ll(vts2tms(rootId, flipY, vts::tileId(range.lod, range.range.ll)));
    auto ur(vts2tms(rootId, flipY, vts::tileId(range.lod, range.range.ur)));
    vts::LodTileRange tr(ll.lod, vts::TileRange(math::InvalidExtents{}));
    update(tr.range, vts::point(ll));
    update(tr.range, vts::point(ur));
    return tr;
}

#endif // mapproxy_support_tms_hpp_included_
