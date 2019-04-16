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

#include "metatile.hpp"
#include "tileindex.hpp"

typedef vts::TileIndex::Flag TiFlag;

void prepareTileIndex(vts::TileIndex &index
                      , const boost::filesystem::path *tilesPath
                      , const Resource &resource
                      , bool navtiles
                      , const MaskTree &maskTree)
{
    // grab and reset tile index
    auto &ti(index);

    // clean tile index
    ti = {};

    // generate tile index bootstrap from lod/tile ranges
    {
        // build default flags
        TiFlag::value_type defaultFlags(TiFlag::mesh);
        if (!tilesPath) {
            // no external tile index, everything is watertight by default
            defaultFlags |= TiFlag::watertight;
        }

        for (auto lod : resource.lodRange) {
            // treat whole lod as a huge metatile and process each block
            // independently
            for (const auto &block
                     : metatileBlocks(resource, vts::TileId(lod), lod, true))
            {
                LOG(info1) << "Generating tile index LOD <" << lod
                           << ">: ancestor: "
                       << block.commonAncestor.nodeId()
                           << ", block: " << block.view << ".";

                if (block.commonAncestor.productive()
                    && in(lod, resource.lodRange))
                {
                    TiFlag::value_type flags(defaultFlags);

                    if (navtiles && (lod == resource.lodRange.min)) {
                        // force navtile in topmost lod
                        flags |= TiFlag::navtile;
                    }

                    // set current block to computed value
                    ti.set(lod, block.view, flags);
                }
            }
        }
    }

    // and clip with dataset tiles
    if (tilesPath) {
        // load definition
        LOG(debug) << "Loading tiling from " << *tilesPath << ".";
        vts::TileIndex datasetTiles;
        datasetTiles.load(*tilesPath);

        if (resource.lodRange.max > datasetTiles.maxLod()) {
            LOG(debug) << "Loaded tiling is too shallow ("
                       << datasetTiles.maxLod() << " vs "
                       << resource.lodRange.max << "); enlarging.";

            // make tiling available from root to resource max LOD and copy
            // tiling data from original to new bottom LOD
            // NB: tiling *should* be from root
            datasetTiles
                .makeAvailable(vts::LodRange(0, resource.lodRange.max))
                .completeDownFromBottom();
        }

        // TODO: unset navtile info if navtiles is true
        auto combiner([&](TiFlag::value_type o, TiFlag::value_type n)
                      -> TiFlag::value_type
        {
            if (!o || !n) {
                // no intersection -> nothing
                return 0;
            }

            // intersection -> merge flags
            return o | n;
        });

        LOG(debug) << "Combining synthetic tileindex with tiling.";
        ti.combine(datasetTiles, combiner, resource.lodRange);
    }

    // finally clip everything by mask tree if present
    if (maskTree) {
        /** TODO: RF partial nodes should be handled differently
         */
        const auto treeDepth(maskTree.depth());
        for (const auto lod : resource.lodRange) {
            auto filterByMask([&](MaskTree::Node node, boost::tribool value)
            {
                // valid -> nothing to be done
                if (value) { return; }

                // update node to match lod grid
                node.shift(treeDepth - lod);

                vts::TileRange tileRange
                    (node.x, node.y
                     , node.x + node.size - 1, node.y + node.size - 1);

                if (!value) {
                    // invalid: unset whole quad
                    ti.set(lod, tileRange, TiFlag::none);
                    return;
                }

                // partially covered tile: unset watertight node; tells the
                // implementation that we are removing some values therefore it
                // is not needed to generate tree for lods that do not exist
                ti.update(lod, tileRange
                          , [&](TiFlag::value_type value)
                {
                    // remove watertight flag
                    return (value & ~TiFlag::watertight);
                }, false);
            });

            maskTree.forEachQuad(filterByMask, MaskTree::Constraints(lod));
        }
    }
}
