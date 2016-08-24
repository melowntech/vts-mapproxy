#ifndef mapproxy_support_metatile_hpp_included_
#define mapproxy_support_metatile_hpp_included_

#include "vts-libs/registry.hpp"
#include "vts-libs/vts/basetypes.hpp"
#include "vts-libs/vts/tileop.hpp"

#include "../resource.hpp"

#include "./coverage.hpp"

namespace vts = vadstena::vts;
namespace vr = vadstena::registry;

/** Metatile block (part of metatile sharing same spatial division)
 */
struct MetatileBlock {
    std::string srs;
    vts::TileRange view;
    math::Extents2 extents;

    /** Common ancestor of nodes in this block
     */
    vts::NodeInfo commonAncestor;

    vts::TileId offset;

    typedef std::vector<MetatileBlock> list;

    MetatileBlock(vts::Lod lod, const vr::ReferenceFrame &referenceFrame
                  , const std::string &srs, const vts::TileRange &view
                  , const math::Extents2 &extents);

    bool valid() const { return commonAncestor.valid(); }
    bool partial() const { return commonAncestor.partial(); }
};

/** Generate metatile blocks for given metatile id in given reference frame
 *
 *  \param referenceFrame reference frame
 *  \param tileId metatile id
 *  \param metaBinaryOrder metatile binary order override if nonzero
 */
MetatileBlock::list metatileBlocks(const Resource &resource
                                   , const vts::TileId &tileId
                                   , unsigned int metaBinaryOrder = 0
                                   , bool includeInvalid = false);

inline bool special(const vr::ReferenceFrame &referenceFrame
                    , const vts::TileId &tileId)
{
    if (const auto *node
        = referenceFrame.find(vts::rfNodeId(tileId), std::nothrow))
    {
        switch (node->partitioning.mode) {
        case vr::PartitioningMode::manual:
            return true;
        default:
            return false;
        }
    }
    return false;
}

class ShiftMask {
public:
    ShiftMask(const MetatileBlock &block, int samplesPerTile
              , const MaskTree &maskTree_ = MaskTree())
        : offset_(block.offset.x * samplesPerTile
                  , block.offset.y * samplesPerTile)
        , mask_(generateCoverage((1 << block.offset.lod) * samplesPerTile
                                 , block.commonAncestor, maskTree_
                                 , vts::NodeInfo::CoverageType::grid))
    {}

    bool operator()(int x, int y) const {
        return mask_.get(x + offset_(0), y + offset_(1));
    }

private:
    const math::Point2i offset_;
    const vts::NodeInfo::CoverageMask mask_;
};

#endif // mapproxy_support_metatile_hpp_included_
