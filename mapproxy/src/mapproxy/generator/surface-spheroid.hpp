#ifndef mapproxy_generator_surface_spheroid_hpp_included_
#define mapproxy_generator_surface_spheroid_hpp_included_

#include "vts-libs/vts/tileset/tilesetindex.hpp"

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

    const resdef::SurfaceSpheroid &definition_;

    vts::tileset::Index index_;
};

} // namespace generator

#endif // mapproxy_generator_surface_spheroid_hpp_included_
