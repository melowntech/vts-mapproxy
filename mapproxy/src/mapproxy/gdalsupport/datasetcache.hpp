#ifndef mapproxy_datasetcache_hpp_included_
#define mapproxy_datasetcache_hpp_included_

#include <map>

#include "geo/geodataset.hpp"

class DatasetCache {
public:
    DatasetCache() : hits_() {}

    geo::GeoDataset& operator()(const std::string &path);

    bool worn();

private:
    typedef std::map<std::string, geo::GeoDataset> Cache;

    Cache datasets_;

    std::size_t hits_;
};

#endif // mapproxy_dataset_hpp_included_
