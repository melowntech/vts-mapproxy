#include "../error.hpp"
#include "./datasetcache.hpp"

geo::GeoDataset& DatasetCache::dataset(const std::string &path)
{
    auto fdatasets(datasets_.find(path));
    if (fdatasets != datasets_.end()) { return fdatasets->second; }

    return datasets_.insert(Cache::value_type
                            (path, geo::GeoDataset::open(path))).first->second;
}

geo::GeoDataset& DatasetCache::mask(const std::string &path)
{
    auto fmasks(masks_.find(path));
    if (fmasks != masks_.end()) { return fmasks->second; }

    // TODO: open with extra options
    auto ds(geo::GeoDataset::open(path));
    switch (ds.type()) {
    case geo::GeoDataset::Type::alpha:
    case geo::GeoDataset::Type::grayscale:
        break;

    default:
        LOGTHROW(warn2, InternalError)
            << "Dataset at <" << path
            << "> is not single a single byte channel one.";
    }

    return masks_.insert(Cache::value_type(path, std::move(ds))).first->second;
}
