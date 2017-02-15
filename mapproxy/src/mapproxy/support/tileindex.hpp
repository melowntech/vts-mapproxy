#ifndef mapproxy_support_tileindex_hpp_included_
#define mapproxy_support_tileindex_hpp_included_

#include "vts-libs/vts/tileindex.hpp"
#include "vts-libs/vts/tileset/tilesetindex.hpp"

#include "../resource.hpp"
#include "./coverage.hpp"

namespace vts = vtslibs::vts;

void prepareTileIndex(vts::TileIndex &index
                      , const boost::filesystem::path &tilesPath
                      , const Resource &resource
                      , bool navtiles = false
                      , const MaskTree &maskTree = MaskTree());

void prepareTileIndex(vts::tileset::Index &index
                      , const boost::filesystem::path &tilesPath
                      , const Resource &resource
                      , bool navtiles = false
                      , const MaskTree &maskTree = MaskTree());

// inlines

inline void prepareTileIndex(vts::tileset::Index &index
                             , const boost::filesystem::path &tilesPath
                             , const Resource &resource
                             , bool navtiles
                             , const MaskTree &maskTree)
{
    prepareTileIndex(index.tileIndex, tilesPath, resource, navtiles, maskTree);
}

#endif // mapproxy_support_tileindex_hpp_included_
