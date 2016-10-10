#ifndef mapproxy_support_geo_hpp_included_
#define mapproxy_support_geo_hpp_included_

#include "geo/geodataset.hpp"

#include "vts-libs/vts/nodeinfo.hpp"

namespace vts = vadstena::vts;

struct DemDataset {
    std::string dataset;
    boost::optional<std::string> geoidGrid;

    DemDataset() = default;

    DemDataset(const std::string &dataset
               , const boost::optional<std::string> &geoidGrid = boost::none)
        : dataset(dataset), geoidGrid(geoidGrid)
    {}

    bool operator==(const DemDataset &other) const;
    bool operator!=(const DemDataset &other) const;

    typedef std::vector<DemDataset> list;
};

double tileCircumference(const math::Extents2 &extents
                         , const geo::SrsDefinition &srs
                         , const geo::GeoDataset &dataset
                         , int samples = 20);

math::Extents2 extentsPlusHalfPixel(const math::Extents2 &extents
                                    , const math::Size2 &pixels);


// inlines

inline bool DemDataset::operator==(const DemDataset &other) const
{
    return ((dataset == other.dataset)
            && (geoidGrid == other.geoidGrid));
}

inline bool DemDataset::operator!=(const DemDataset &other) const
{
    return !operator==(other);
}

#endif // mapproxy_support_geo_hpp_included_
