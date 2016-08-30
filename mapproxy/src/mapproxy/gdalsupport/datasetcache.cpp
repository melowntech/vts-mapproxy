#include "../error.hpp"
#include "./datasetcache.hpp"

geo::GeoDataset& DatasetCache::operator()(const std::string &path)
{
    ++hits_;

    auto fdatasets(datasets_.find(path));
    if (fdatasets != datasets_.end()) { return fdatasets->second; }

    return datasets_.insert(Cache::value_type
                            (path, geo::GeoDataset::open(path))).first->second;
}

bool DatasetCache::worn()
{
    return false;
}
