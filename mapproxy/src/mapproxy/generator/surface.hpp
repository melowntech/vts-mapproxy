/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef mapproxy_generator_surface_hpp_included_
#define mapproxy_generator_surface_hpp_included_

#include <boost/optional.hpp>

#include "vts-libs/registry/extensions.hpp"
#include "vts-libs/vts/tileset/properties.hpp"
#include "vts-libs/vts/mesh.hpp"

#include "../heightfunction.hpp"
#include "../support/mesh.hpp"
#include "../support/mmapped/tilesetindex.hpp"
#include "../generator.hpp"
#include "../definition.hpp"

namespace vts = vtslibs::vts;
namespace vre = vtslibs::registry::extensions;

namespace generator {

class SurfaceBase : public Generator {
public:
    SurfaceBase(const Params &params);

    typedef resource::Surface Definition;

protected:
    boost::filesystem::path filePath(vts::File fileType) const;
    bool updateProperties(const Definition &def);
    bool loadFiles(const Definition &definition);

    vts::ExtraTileSetProperties extraProperties(const Definition &def)
        const;

protected:
    const vre::Tms& getTms() const;

    boost::optional<mmapped::Index> index_;
    vts::FullTileSetProperties properties_;

private:
    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , Sink &sink) const;

    virtual void generateMetatile(const vts::TileId &tileId
                                  , Sink &sink
                                  , const SurfaceFileInfo &fileInfo
                                  , Arsenal &arsenal) const = 0;

    void generateMesh(const vts::TileId &tileId
                      , Sink &sink
                      , const SurfaceFileInfo &fileInfo
                      , Arsenal &arsenal) const;

    void generate2dMask(const vts::TileId &tileId
                        , Sink &sink
                        , const SurfaceFileInfo &fileInfo
                        , Arsenal &arsenal) const;

    void generate2dMetatile(const vts::TileId &tileId
                            , Sink &sink
                            , const SurfaceFileInfo &fileInfo
                            , Arsenal &arsenal) const;

    void generateCredits(const vts::TileId &tileId
                         , Sink &sink
                         , const SurfaceFileInfo &fileInfo
                         , Arsenal &arsenal) const;

    virtual void generateNavtile(const vts::TileId &tileId
                                 , Sink &sink
                                 , const SurfaceFileInfo &fileInfo
                                 , Arsenal &arsenal) const = 0;

    void generateDebugNode(const vts::TileId &tileId
                           , Sink &sink
                           , const SurfaceFileInfo &fileInfo
                           , Arsenal &arsenal) const;

    /** Generic mesh generation, used by both surface and terrain interface.
     */
    virtual AugmentedMesh
    generateMeshImpl(const vts::NodeInfo &nodeInfo
                     , Sink &sink, Arsenal &arsenal
                     , const OptHeight &defaultHeight = boost::none) const = 0;

    /** Cesium terrain provider support. Generates non-VTS mesh.
     */

    Task terrainInterface(const FileInfo &fileInfo, Sink &sink) const;

    void generateTerrain(const vts::TileId &tileId
                         , Sink &sink
                         , const TerrainFileInfo &fileInfo
                         , Arsenal &arsenal, const vre::Tms &tms) const;

    void layerJson(Sink &sink, const TerrainFileInfo &fileInfo
                   , const vre::Tms &tms) const;

    void cesiumConf(Sink &sink, const TerrainFileInfo &fileInfo
                    , const vre::Tms &tms) const;

    std::string cesiumReadme() const;

    const Definition &definition_;
    const vre::Tms *tms_;
};

} // namespace generator

#endif // mapproxy_generator_surface_hpp_included_
