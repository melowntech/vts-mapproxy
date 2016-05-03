#ifndef mapproxy_generator_surface_spheroid_hpp_included_
#define mapproxy_generator_surface_spheroid_hpp_included_

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

    boost::filesystem::path filePath(vts::File fileType) const;

    const resdef::SurfaceDem &definition_;

    /** Path to original dataset (must contain overviews)
     */
    const std::string dataset_;

    /** Path to special dataset with overviews created with minimum filter
     */
    const std::string datasetMin_;

    /** Path to special dataset with overviews created with maximum filter
     */
    const std::string datasetMax_;

    vts::tileset::Index index_;
    vts::FullTileSetProperties properties_;
};

} // namespace generator

#endif // mapproxy_generator_surface_spheroid_hpp_included_
