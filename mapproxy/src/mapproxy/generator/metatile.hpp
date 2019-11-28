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

#ifndef mapproxy_metatile_hpp_included_
#define mapproxy_metatile_hpp_included_

#include "vts-libs/vts/tileindex.hpp"
#include "vts-libs/vts/metatile.hpp"

#include "../support/coverage.hpp"
#include "../support/mmapped/tileindex.hpp"

#include "../generator.hpp"

#include "../heightfunction.hpp"

struct MetatileOverrides {
    enum class CreditsMode { replace, add };

    vts::SubMesh::TextureMode textureMode;
    DualId::set credits;
    CreditsMode creditsMode;

    MetatileOverrides()
        : textureMode(vts::SubMesh::external)
        , creditsMode(CreditsMode::add)
    {}

    MetatileOverrides(vts::SubMesh::TextureMode textureMode)
        : textureMode(textureMode)
        , creditsMode(CreditsMode::add)

    {}

    void addCredits(const DualId::set &addition);

    DualId::set mergedCredits(const DualId::set &original) const;

    vr::IdSet mergedCredits(const vr::IdSet &original) const;
};

vts::MetaTile metatileFromDem(const vts::TileId &tileId, Sink &sink
                              , Arsenal &arsenal
                              , const Resource &resource
                              , const vts::TileIndex &tileIndex
                              , const std::string &demDataset
                              , const boost::optional<std::string> &geoidGrid
                              , const MaskTree &maskTree = MaskTree()
                              , const boost::optional<int> &displaySize
                              = boost::none
                              , const HeightFunction::pointer &heightFunction
                              = HeightFunction::pointer()
                              , const MetatileOverrides &overrides
                              = MetatileOverrides());

vts::MetaTile metatileFromDem(const vts::TileId &tileId, Sink &sink
                              , Arsenal &arsenal
                              , const Resource &resource
                              , const mmapped::TileIndex &tileIndex
                              , const std::string &demDataset
                              , const boost::optional<std::string> &geoidGrid
                              , const MaskTree &maskTree = MaskTree()
                              , const boost::optional<int> &displaySize
                              = boost::none
                              , const HeightFunction::pointer &heightFunction
                              = HeightFunction::pointer()
                              , const MetatileOverrides &overrides
                              = MetatileOverrides());

// inines

inline DualId::set
MetatileOverrides::mergedCredits(const DualId::set &original) const
{
    // just send configured credits
    if (creditsMode == CreditsMode::replace) { return credits; }

    // merge original with configured credits
    auto c(original);
    c.insert(credits.begin(), credits.end());
    return c;
}

inline vr::IdSet
MetatileOverrides::mergedCredits(const vr::IdSet &original) const
{
    const auto thisCredits(asIntSet(credits));
    // just send configured credits
    if (creditsMode == CreditsMode::replace) { return thisCredits; }

    // merge original with configured credits
    auto c(original);
    c.insert(thisCredits.begin(), thisCredits.end());
    return c;
}

inline void MetatileOverrides::addCredits(const DualId::set &addition)
{
    credits.insert(addition.begin(), addition.end());
}

#endif // mapproxy_metatile_hpp_included_
