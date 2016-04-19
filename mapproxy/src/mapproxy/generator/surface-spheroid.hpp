#ifndef mapproxy_generator_surface_spheroid_hpp_included_
#define mapproxy_generator_surface_spheroid_hpp_included_

#include "vts-libs/vts/tileset/tilesetindex.hpp"
#include "vts-libs/vts/tileset/properties.hpp"

#include "../generator.hpp"

namespace vts = vadstena::vts;

namespace generator {

class SurfaceSpheroid : public Generator {
public:
    SurfaceSpheroid(const Config &config, const Resource &resource);

private:
    virtual void prepare_impl();
    virtual vts::MapConfig
    mapConfig_impl(ResourceRoot root) const;

    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , const Sink::pointer &sink) const;

    void generateMetatile(const vts::TileId &tileId
                          , const Sink::pointer &sink
                          , GdalWarper &warper) const;

    const resdef::SurfaceSpheroid &definition_;

    vts::tileset::Index index_;
    vts::FullTileSetProperties properties_;
};

} // namespace generator

#endif // mapproxy_generator_surface_spheroid_hpp_included_
