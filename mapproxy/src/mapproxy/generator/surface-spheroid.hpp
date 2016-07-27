#ifndef mapproxy_generator_surface_spheroid_hpp_included_
#define mapproxy_generator_surface_spheroid_hpp_included_

#include "vts-libs/vts/tileset/tilesetindex.hpp"
#include "vts-libs/vts/tileset/properties.hpp"

#include "./surface.hpp"

namespace vts = vadstena::vts;

namespace generator {

class SurfaceSpheroid : public SurfaceBase {
public:
    SurfaceSpheroid(const Params &params);

    struct Definition : public DefinitionBase {
        unsigned int textureLayerId;
        boost::optional<std::string> geoidGrid;

        Definition() : textureLayerId() {}
        bool operator==(const Definition &o) const;

    private:
        virtual void from_impl(const boost::any &value);
        virtual void to_impl(boost::any &value) const;
        virtual bool same_impl(const DefinitionBase &other) const {
            return (*this == other.as<Definition>());
        }
    };

private:
    virtual void prepare_impl();
    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const;

    virtual void generateMetatile(const vts::TileId &tileId
                                  , Sink &sink
                                  , const SurfaceFileInfo &fileInfo
                                  , Arsenal &arsenal) const;

    virtual vts::Mesh generateMeshImpl(const vts::NodeInfo &nodeInfo
                                       , Sink &sink
                                       , const SurfaceFileInfo &fileInfo
                                       , Arsenal &arsenal
                                       , bool withMask) const;

    virtual void generateNavtile(const vts::TileId &tileId
                                 , Sink &sink
                                 , const SurfaceFileInfo &fileInfo
                                 , Arsenal &arsenal) const;

    const Definition &definition_;
};

} // namespace generator

#endif // mapproxy_generator_surface_spheroid_hpp_included_
