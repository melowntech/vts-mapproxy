#ifndef mapproxy_generator_surface_spheroid_hpp_included_
#define mapproxy_generator_surface_spheroid_hpp_included_

#include "imgproc/rastermask/mappedqtree.hpp"

#include "vts-libs/vts/tileset/tilesetindex.hpp"
#include "vts-libs/vts/tileset/properties.hpp"

#include "./surface.hpp"

namespace vts = vadstena::vts;

namespace generator {

class SurfaceDem : public SurfaceBase {
public:
    SurfaceDem(const Config &config, const Resource &resource);

private:
    virtual void prepare_impl();
    virtual vts::MapConfig
    mapConfig_impl(ResourceRoot root) const;

    void generateMetatile(const vts::TileId &tileId
                          , const Sink::pointer &sink
                          , const SurfaceFileInfo &fileInfo
                          , GdalWarper &warper) const;

    void generateMesh(const vts::TileId &tileId
                      , const Sink::pointer &sink
                      , const SurfaceFileInfo &fileInfo
                      , GdalWarper &warper) const;

    void generateNavtile(const vts::TileId &tileId
                         , const Sink::pointer &sink
                         , const SurfaceFileInfo &fileInfo
                         , GdalWarper &warper) const;

    const resdef::SurfaceDem &definition_;

    /** Path to original dataset (must contain overviews)
     */
    const std::string dataset_;

    vts::tileset::Index index_;
    vts::FullTileSetProperties properties_;

    // mask tree
    typedef imgproc::mappedqtree::RasterMask MaskTree;
    MaskTree maskTree_;
};

} // namespace generator

#endif // mapproxy_generator_surface_spheroid_hpp_included_
