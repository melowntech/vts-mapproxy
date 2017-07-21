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

#ifndef mapproxy_support_mmapped_tilesetindex_hpp_included_
#define mapproxy_support_mmapped_tilesetindex_hpp_included_

#include "./tileindex.hpp"

namespace mmapped {

class Index {
public:
    typedef std::shared_ptr<Index> pointer;

    Index(unsigned int metaBinaryOrder, const boost::filesystem::path &path)
        : tileIndex(path)
        , metaBinaryOrder_(metaBinaryOrder)
    {}

    /** Tile index (tile data presence flags)
     */
    const TileIndex tileIndex;

    bool check(const vts::TileId &tileId, vts::TileFile type) const;

    bool real(const vts::TileId &tileId) const;

    bool meta(const vts::TileId &tileId) const;

    /** Checks file type and returns flags in case of match.
     */
    vts::TileIndex::Flag::value_type
    checkAndGetFlags(const vts::TileId &tileId, vts::TileFile type) const;

    unsigned int metaBinaryOrder() const { return metaBinaryOrder_; }

private:
    unsigned int metaBinaryOrder_;
};

// inlines

inline bool Index::real(const vts::TileId &tileId) const
{
    return (tileIndex.get(tileId) & vts::TileIndex::Flag::real);

}

} // namespace mmapped

#endif // mapproxy_support_mmapped_tilesetindex_hpp_included_
