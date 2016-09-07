#ifndef mapproxy_generator_geodata_vector_tiled_hpp_included_
#define mapproxy_generator_geodata_vector_tiled_hpp_included_

#include "geo/geodataset.hpp"

#include "vts-libs/vts/urltemplate.hpp"
#include "vts-libs/vts/tileset/tilesetindex.hpp"

#include "./geodatavectorbase.hpp"

#include "../support/demconfig.hpp"

namespace generator {

class GeodataVectorTiled : public GeodataVectorBase {
public:
    GeodataVectorTiled(const Params &params);

private:
    virtual void prepare_impl(Arsenal &arsenal);

    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const;
    virtual vr::FreeLayer freeLayer_impl(ResourceRoot root) const;

    virtual void generateMetatile(Sink &sink
                                  , const GeodataFileInfo &fileInfo
                                  , Arsenal &arsenal) const;

    virtual void generateGeodata(Sink &sink
                                 , const GeodataFileInfo &fileInfo
                                 , Arsenal &arsenal) const;

    Definition definition_;

    /** Path to /dem dataset
     */
    const std::string demDataset_;

    DemConfig demConfig_;
    geo::GeoDataset::Descriptor dem_;
    double effectiveGsdArea_;
    bool effectiveGsdAreaComputed_;

    vts::UrlTemplate tileUrl_;

    const vr::Srs &physicalSrs_;

    vts::tileset::Index index_;
};

} // namespace generator

#endif // mapproxy_generator_geodata_vector_tiled_hpp_included_
