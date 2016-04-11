#ifndef mapproxy_datasetcache_hpp_included_
#define mapproxy_datasetcache_hpp_included_

#include <map>

#include "geo/geodataset.hpp"

class DatasetCache {
public:
    DatasetCache() {}

    geo::GeoDataset& dataset(const std::string &path);
    geo::GeoDataset& mask(const std::string &path);

private:
    typedef std::map<std::string, geo::GeoDataset> Cache;

    Cache datasets_;
    Cache masks_;
};

#endif // mapproxy_dataset_hpp_included_
