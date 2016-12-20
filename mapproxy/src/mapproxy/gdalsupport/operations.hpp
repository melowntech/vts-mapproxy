#ifndef mapproxy_gdalsupport_operations_hpp_included_
#define mapproxy_gdalsupport_operations_hpp_included_

#include "../gdalsupport.hpp"
#include "./types.hpp"
#include "datasetcache.hpp"

cv::Mat* warp(DatasetCache &cache, ManagedBuffer &mb
              , const GdalWarper::RasterRequest &req);

GdalWarper::Heightcoded*
heightcode(DatasetCache &cache, ManagedBuffer &mb
           , const std::string &vectorDs
           , const DemDataset::list &rasterDs
           , geo::heightcoding::Config config
           , const boost::optional<std::string> &vectorGeoidGrid);

GdalWarper::Heightcoded*
heightcode(DatasetCache &cache, ManagedBuffer &mb
           , const std::string &vectorDs
           , const GdalWarper::Navtile &navtile
           , const ConstBlock &dataBlock
           , geo::heightcoding::Config config
           , const std::string &fallbackDs
           , const boost::optional<std::string> &geoidGrid);

#endif // mapproxy_gdalsupport_operations_hpp_included_
