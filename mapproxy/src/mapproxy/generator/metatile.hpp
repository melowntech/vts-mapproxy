#ifndef mapproxy_metatile_hpp_included_
#define mapproxy_metatile_hpp_included_

#include "vts-libs/vts/tileindex.hpp"
#include "vts-libs/vts/metatile.hpp"

#include "../support/coverage.hpp"

#include "../generator.hpp"

vts::MetaTile metatileFromDem(const vts::TileId &tileId, Sink &sink
                              , Arsenal &arsenal
                              , const Resource &resource
                              , const vts::TileIndex &tileIndex
                              , const std::string &demDataset
                              , const boost::optional<std::string> &geoidGrid
                              , const MaskTree &maskTree = MaskTree());

#endif // mapproxy_metatile_hpp_included_
