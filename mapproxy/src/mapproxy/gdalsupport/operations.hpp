#ifndef mapproxy_gdalsupport_operations_hpp_included_
#define mapproxy_gdalsupport_operations_hpp_included_

#include "../gdalsupport.hpp"
#include "./types.hpp"
#include "datasetcache.hpp"

cv::Mat* warp(DatasetCache &cache, ManagedBuffer &mb
              , const GdalWarper::RasterRequest &req);

GdalWarper::Heighcoded*
heightcode(DatasetCache &cache, ManagedBuffer &mb
           , const std::string &vectorDs
           , const std::string &rasterDs
           , geo::heightcoding::Config config
           , const boost::optional<std::string> &geoidGrid);

#endif // mapproxy_gdalsupport_operations_hpp_included_
