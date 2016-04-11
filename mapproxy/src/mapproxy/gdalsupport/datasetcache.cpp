#include "./datasetcache.hpp"

geo::GeoDataset& DatasetCache::get(const std::string &path)
{
    auto fcache(cache_.find(path));
    if (fcache != cache_.end()) { return fcache->second; }

    return cache_.insert(Cache::value_type
                         (path, geo::GeoDataset::open(path))).first->second;
}
