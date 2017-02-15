#ifndef mapproxy_generator_surface_spheroid_hpp_included_
#define mapproxy_generator_surface_spheroid_hpp_included_

#include "imgproc/rastermask/mappedqtree.hpp"

#include "vts-libs/vts/tileset/tilesetindex.hpp"
#include "vts-libs/vts/tileset/properties.hpp"

#include "./surface.hpp"

#include "../support/coverage.hpp"

namespace vts = vtslibs::vts;
namespace vr = vtslibs::registry;

namespace generator {

class SurfaceDem : public SurfaceBase {
public:
    SurfaceDem(const Params &params);

    ~SurfaceDem();

    struct Definition : public SurfaceBase::SurfaceDefinition {
        DemDataset dem;
        boost::optional<boost::filesystem::path> mask;
        unsigned int textureLayerId;
        boost::optional<std::string> heightcodingAlias;

        Definition() : textureLayerId() {}

    private:
        virtual void from_impl(const boost::any &value);
        virtual void to_impl(boost::any &value) const;
        virtual Changed changed_impl(const DefinitionBase &other) const;
    };

private:
    virtual void prepare_impl(Arsenal &arsenal);
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

    vts::MetaTile generateMetatileImpl(const vts::TileId &tileId
                                       , Sink &sink
                                       , Arsenal &arsenal) const;

    void addToRegistry();

    void removeFromRegistry();

    const Definition &definition_;

    /** Path to original dataset (must contain overviews)
     */
    const DemDataset dem_;

    // mask tree
    MaskTree maskTree_;
};

} // namespace generator

#endif // mapproxy_generator_surface_spheroid_hpp_included_
