#ifndef mapproxy_support_tileindex_hpp_included_
#define mapproxy_support_tileindex_hpp_included_

#include "vts-libs/vts/tileset/tilesetindex.hpp"

#include "../resource.hpp"
#include "./coverage.hpp"

namespace vts = vadstena::vts;

void prepareTileIndex(vts::tileset::Index &index
                      , const boost::filesystem::path &tilesPath
                      , const Resource &resource
                      , bool navtiles = false
                      , const MaskTree &maskTree = MaskTree());

#endif // mapproxy_support_tileindex_hpp_included_
