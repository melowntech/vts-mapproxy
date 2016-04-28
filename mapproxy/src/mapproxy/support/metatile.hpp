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
};

/** Generate metatile blocks for given metatile id in given reference frame
 *
 *  \param referenceFrame reference frame
 *  \param tileId metatile id
 *  \param metaBinaryOrder metatile binary order override if nonzero
 */
MetatileBlock::list metatileBlocks(const Resource &resource
                                   , const vts::TileId &tileId
                                   , unsigned int metaBinaryOrder = 0);

#endif // mapproxy_metatile_included_
