#ifndef mapproxy_support_geo_hpp_included_
#define mapproxy_support_geo_hpp_included_

#include "geo/geodataset.hpp"

#include "vts-libs/vts/nodeinfo.hpp"

namespace vts = vadstena::vts;

double tileCircumference(const math::Extents2 &extents
                         , const geo::SrsDefinition &srs
                         , const geo::GeoDataset &dataset
                         , int samples = 20);

#endif // mapproxy_support_geo_hpp_included_
