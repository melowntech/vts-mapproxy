#ifndef mapproxy_generator_surface_spheroid_hpp_included_
#define mapproxy_generator_surface_spheroid_hpp_included_

#include "imgproc/rastermask/mappedqtree.hpp"

#include "vts-libs/vts/tileset/tilesetindex.hpp"
#include "vts-libs/vts/tileset/properties.hpp"

#include "./surface.hpp"

#include "../support/coverage.hpp"

namespace vts = vadstena::vts;

namespace generator {

class SurfaceDem : public SurfaceBase {
public:
    SurfaceDem(const Config &config, const Resource &resource);

private:
    virtual void prepare_impl();
    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const;

    virtual void generateMetatile(const vts::TileId &tileId
                                  , Sink &sink
                                  , const SurfaceFileInfo &fileInfo
                                  , GdalWarper &warper) const;

    virtual vts::Mesh generateMeshImpl(const vts::NodeInfo &nodeInfo
                                       , Sink &sink
                                       , const SurfaceFileInfo &fileInfo
                                       , GdalWarper &warper
                                       , bool withMask) const;

    virtual void generateNavtile(const vts::TileId &tileId
                                 , Sink &sink
                                 , const SurfaceFileInfo &fileInfo
                                 , GdalWarper &warper) const;

    vts::MetaTile generateMetatileImpl(const vts::TileId &tileId
                                       , Sink &sink
                                       , GdalWarper &warper) const;

    const resdef::SurfaceDem &definition_;

    /** Path to original dataset (must contain overviews)
     */
    const std::string dataset_;

    // mask tree
    MaskTree maskTree_;
};

} // namespace generator

#endif // mapproxy_generator_surface_spheroid_hpp_included_
