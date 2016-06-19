#ifndef mapproxy_generator_surface_spheroid_hpp_included_
#define mapproxy_generator_surface_spheroid_hpp_included_

#include "vts-libs/vts/tileset/tilesetindex.hpp"
#include "vts-libs/vts/tileset/properties.hpp"

#include "./surface.hpp"

namespace vts = vadstena::vts;

namespace generator {

class SurfaceSpheroid : public SurfaceBase {
public:
    SurfaceSpheroid(const Config &config, const Resource &resource);

private:
    virtual void prepare_impl();
    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const;

    virtual void generateMetatile(const vts::TileId &tileId
                                  , const Sink::pointer &sink
                                  , const SurfaceFileInfo &fileInfo
                                  , GdalWarper &warper) const;

    virtual vts::Mesh generateMeshImpl(const vts::NodeInfo &nodeInfo
                                       , const Sink::pointer &sink
                                       , const SurfaceFileInfo &fileInfo
                                       , GdalWarper &warper
                                       , bool withMask) const;

    virtual void generateNavtile(const vts::TileId &tileId
                                 , const Sink::pointer &sink
                                 , const SurfaceFileInfo &fileInfo
                                 , GdalWarper &warper) const;

    virtual void generate2dMetatile(const vts::TileId &tileId
                                    , const Sink::pointer &sink
                                    , const SurfaceFileInfo &fileInfo
                                    , GdalWarper &warper) const;

    virtual void generate2dCredits(const vts::TileId &tileId
                                   , const Sink::pointer &sink
                                   , const SurfaceFileInfo &fileInfo
                                   , GdalWarper &warper) const;

    const resdef::SurfaceSpheroid &definition_;
};

} // namespace generator

#endif // mapproxy_generator_surface_spheroid_hpp_included_
