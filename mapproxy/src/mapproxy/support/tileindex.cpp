#include "./metatile.hpp"
#include "./tileindex.hpp"

typedef vts::TileIndex::Flag TiFlag;

void prepareTileIndex(vts::TileIndex &index
                      , const boost::filesystem::path &tilesPath
                      , const Resource &resource
                      , bool navtiles
                      , const MaskTree &maskTree)
{
    // load definition
    vts::TileIndex datasetTiles;
    datasetTiles.load(tilesPath);

    // grab and reset tile index
    auto &ti(index);

    // clean tile index
    ti = {};

    // generate tile index from lod/tile ranges
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
                TiFlag::value_type flags(TiFlag::mesh);

                if (navtiles && (lod == resource.lodRange.min)) {
                    // force navtile in topmost lod
                    flags |= TiFlag::navtile;
                }

                // set current block to computed value
                ti.set(lod, block.view, flags);
            }
        }
    }

    // and clip with dataset tiles
    {
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
