#ifndef mapproxy_metatile_hpp_included_
#define mapproxy_metatile_hpp_included_

#include "vts-libs/registry.hpp"
#include "vts-libs/vts/basetypes.hpp"

#include "../resource.hpp"

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

/** Same as above but without any limits
 */
MetatileBlock::list metatileBlocks(const vr::ReferenceFrame &referenceFrame
                                   , const vts::TileId &tileId
                                   , unsigned int metaBinaryOrder = 0
                                   , bool includeInvalid = false);

class ShiftMask {
public:
    ShiftMask(const MetatileBlock &block, int samplesPerTile)
        : offset_(block.offset)
        , size_((1 << offset_.lod) * samplesPerTile + 1
                , (1 << offset_.lod) * samplesPerTile + 1)
        , mask_(block.commonAncestor.coverageMask
                (vts::NodeInfo::CoverageType::grid, size_, 1))
    {}

    bool operator()(int x, int y) const {
        return mask_.get(x + offset_.x, y + offset_.y);
    }

private:
    const vts::TileId offset_;
    const math::Size2 size_;
    const vts::NodeInfo::CoverageMask mask_;
};

#endif // mapproxy_metatile_included_
