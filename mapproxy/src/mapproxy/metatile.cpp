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

    // compose extents
    math::Extents2 extents(llNode.extents().ll(0), urNode.extents().ll(1)
                           , urNode.extents().ur(0), llNode.extents().ur(1));

    // TODO: break into multiple blocks if multiple SRS are present here
    return { MetatileBlock(llNode.srs(), view, extents) };
}
