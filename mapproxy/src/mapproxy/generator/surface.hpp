#ifndef mapproxy_generator_surface_hpp_included_
#define mapproxy_generator_surface_hpp_included_

#include "vts-libs/vts/tileset/tilesetindex.hpp"
#include "vts-libs/vts/tileset/properties.hpp"
#include "vts-libs/vts/mesh.hpp"

#include "../generator.hpp"

namespace vts = vadstena::vts;

namespace generator {

class SurfaceBase : public Generator {
public:
    SurfaceBase(const Config &config, const Resource &resource);

protected:
    boost::filesystem::path filePath(vts::File fileType) const;

protected:
    vts::tileset::Index index_;
    vts::FullTileSetProperties properties_;

private:
    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , Sink &sink) const;

    virtual void generateMetatile(const vts::TileId &tileId
                                  , Sink &sink
                                  , const SurfaceFileInfo &fileInfo
                                  , GdalWarper &warper) const = 0;

    void generateMesh(const vts::TileId &tileId
                      , Sink &sink
                      , const SurfaceFileInfo &fileInfo
                      , GdalWarper &warper) const;

    void generate2dMask(const vts::TileId &tileId
                        , Sink &sink
                        , const SurfaceFileInfo &fileInfo
                        , GdalWarper &warper) const;

    void generate2dMetatile(const vts::TileId &tileId
                            , Sink &sink
                            , const SurfaceFileInfo &fileInfo
                            , GdalWarper &warper) const;

    void generateCredits(const vts::TileId &tileId
                         , Sink &sink
                         , const SurfaceFileInfo &fileInfo
                         , GdalWarper &warper) const;

    virtual void generateNavtile(const vts::TileId &tileId
                                 , Sink &sink
                                 , const SurfaceFileInfo &fileInfo
                                 , GdalWarper &warper) const = 0;

    enum MeshRequest { full, mesh, mask };

    virtual vts::Mesh generateMeshImpl(const vts::NodeInfo &nodeInfo
                                       , Sink &sink
                                       , const SurfaceFileInfo &fileInfo
                                       , GdalWarper &warper
                                       , bool withMesh) const = 0;
};

} // namespace generator

#endif // mapproxy_generator_surface_hpp_included_
