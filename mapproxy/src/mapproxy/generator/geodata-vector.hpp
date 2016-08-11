#ifndef mapproxy_generator_geodata_vector_hpp_included_
#define mapproxy_generator_geodata_vector_hpp_included_

#include "vts-libs/vts/urltemplate.hpp"
#include "vts-libs/vts/tileset/tilesetindex.hpp"

#include "./geodatavectorbase.hpp"

namespace generator {

class GeodataVector : public GeodataVectorBase {
public:
    GeodataVector(const Params &params);

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

    /** Output metadata.
     */
    geo::heightcoding::Metadata metadata_;

    /** Path to cached output data.
     */
    boost::filesystem::path dataPath_;
};

} // namespace generator

#endif // mapproxy_generator_geodata_vector_hpp_included_
