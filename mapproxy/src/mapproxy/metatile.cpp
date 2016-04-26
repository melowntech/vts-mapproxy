#include <deque>

#include "utility/raise.hpp"

#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/tileop.hpp"

#include "./error.hpp"
#include "./metatile.hpp"

MetatileBlock::list metatileBlocks(const Resource &resource
                                   , const vts::TileId &tileId
                                   , unsigned int metaBinaryOrder)
{
    const auto &referenceFrame(*resource.referenceFrame);
    if (!metaBinaryOrder) {
        // no override, use order from reference frame
        metaBinaryOrder = referenceFrame.metaBinaryOrder;
    }

    const math::Size2 metaSize(1 << metaBinaryOrder, 1 << metaBinaryOrder);
    const unsigned int metaMask(~(metaSize.width - 1));

    if (((tileId.x & metaMask) != tileId.x)
        || ((tileId.y & metaMask) != tileId.y))
    {
        throw utility::makeError<NotFound>
            ("TileId doesn't point to metatile origin.");
    }

    // generate tile range (inclusive!)
    vts::TileRange tr(tileId.x, tileId.y
                      , tileId.x + metaSize.width - 1
                      , tileId.y + metaSize.height - 1);

    // get maximum tile index at this lod
    auto maxIndex(vts::tileCount(tileId.lod) - 1);

    // and clip
    if (tr.ur(0) > maxIndex) { tr.ur(0) = maxIndex; }
    if (tr.ur(1) > maxIndex) { tr.ur(1) = maxIndex; }

    // calculate tile range at current LOD from resource definition
    auto tileRange(vts::childRange(resource.tileRange
                                   , tileId.lod - resource.lodRange.min));

    // check for overlap with defined tile size
    if (!vts::tileRangesOverlap(tileRange, tr)) {
        return {};
    }

    // calculate overlap
    auto view(vts::tileRangesIntersect(tileRange, tr));

    auto llId(vts::tileId(tileId.lod, view.ll));
    auto urId(vts::tileId(tileId.lod, view.ur));

    // grab nodes at opposite sides
    vts::NodeInfo llNode(referenceFrame, llId);
    vts::NodeInfo urNode(referenceFrame, urId);

    if (llNode.subtree() == urNode.subtree()) {
        // whole range resides under same subtree
        // compose extents
        math::Extents2 extents
            (llNode.extents().ll(0), urNode.extents().ll(1)
             , urNode.extents().ur(0), llNode.extents().ur(1));

        // done
        return { MetatileBlock(referenceFrame, llNode.srs(), view, extents) };
    }

    MetatileBlock::list blocks;

    // seed queue of nodes to inspect with ll node
    std::deque<vts::NodeInfo> queue(1, llNode);

    auto push([&](unsigned int x, unsigned int y)
    {
        if ((x <= view.ur(0)) && (y <= view.ur(1))) {
            // push node, masked node is not invalidated
            vts::NodeInfo ni(referenceFrame, vts::TileId(tileId.lod, x, y)
                             , false);
            // LOG(info4) << "push(" << x << ", " << y << ")";
            queue.push_back(ni);
        }
    });

    // process nodes in the queue until empty
    while (!queue.empty()) {
        // pop
        const auto node(queue.front());
        queue.pop_front();
        // LOG(info4) << "Processing " << node.nodeId() << ".";

        // grab stuff
        const auto &rootId(node.subtree().id());

        // LOG(info4) << "rootId " << rootId << ".";
        // LOG(info4) << "rootId.lod: " << rootId.lod;
        // LOG(info4) << "tileId.lod: " << tileId.lod;

        // compute tile range covered by root at current lod
        auto blockRange(vts::childRange
                        (vts::TileRange(rootId.x, rootId.y, rootId.x, rootId.y)
                         , tileId.lod - rootId.lod));
        // LOG(info4) << "blockRange: " << blockRange;

        // now, clip it by view
        auto blockView(vts::tileRangesIntersect(view, blockRange));

        auto blockUrId(vts::tileId(tileId.lod, blockView.ur));
        vts::NodeInfo blockUrNode(referenceFrame, blockUrId
                                  , false);

        // compose extents
        math::Extents2 blockExtents
            (node.extents().ll(0), blockUrNode.extents().ll(1)
             , blockUrNode.extents().ur(0), node.extents().ur(1));

        // new block
        if (node.valid()) {
            blocks.emplace_back
                (referenceFrame, node.srs(), blockView, blockExtents);
        }

        // remember 2 new nodes to check
        push(blockView.ll(0), blockView.ur(1) + 1); // left/bottom
        push(blockView.ur(0) + 1, blockView.ll(1)); // right/top
    }

    // done
    return blocks;
}
