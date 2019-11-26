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

#ifndef mapproxy_generator_providers_hpp_included_
#define mapproxy_generator_providers_hpp_included_

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

class VtsTilesetProvider {
public:
    virtual ~VtsTilesetProvider() {}

    /** Generates mesh and sends it to the output.
     */
    Generator::Task generateMesh(const vts::TileId &tileId
                                 , Sink &sink
                                 , const SurfaceFileInfo &fileInfo
                                 , vts::SubMesh::TextureMode textureMode
                                 = vts::SubMesh::external) const;

    /** Alias for surface file generation.
     */
    Generator::Task generateFile(const FileInfo &fileInfo, Sink sink) const;

    /** Returns path to VTS file if supported.
     */
    boost::optional<boost::filesystem::path> path(vts::File file) const;

    /** Returns tileset properties.
     */
    vts::FullTileSetProperties properties() const;

private:
    virtual Generator::Task
    generateMesh_impl(const vts::TileId &tileId
                      , Sink &sink
                      , const SurfaceFileInfo &fileInfo
                      , vts::SubMesh::TextureMode textureMode)
        const = 0;

    virtual Generator::Task
    generateFile_impl(const FileInfo &fileInfo, Sink sink) const = 0;

    virtual boost::optional<boost::filesystem::path> path_impl(vts::File file)
        const = 0;

    virtual vts::FullTileSetProperties properties_impl() const = 0;
};

class VtsAtlasProvider {
public:
    virtual ~VtsAtlasProvider() {}

    /** Generates image/atlas and sends it to the output.
     */
    Generator::Task generateAtlas(const vts::TileId &tileId, Sink &sink
                                  , const Sink::FileInfo &sfi
                                  , bool atlas = false) const;

private:
    /** Generates image/atlas and sends it to the output.
     */
    virtual Generator::Task
    generateAtlas_impl(const vts::TileId &tileId, Sink &sink
                       , const Sink::FileInfo &sfi
                       , bool atlas = false) const = 0;
};

// inlines

inline Generator::Task
VtsTilesetProvider::generateMesh(const vts::TileId &tileId
                                 , Sink &sink
                                 , const SurfaceFileInfo &fileInfo
                                 , vts::SubMesh::TextureMode textureMode)
    const
{
    return generateMesh_impl(tileId, sink, fileInfo, textureMode);
}

inline Generator::Task
VtsTilesetProvider::generateFile(const FileInfo &fileInfo, Sink sink) const
{
    return generateFile_impl(fileInfo, sink);
}

inline boost::optional<boost::filesystem::path>
VtsTilesetProvider::path(vts::File file) const
{
    return path_impl(file);
}

inline vts::FullTileSetProperties VtsTilesetProvider::properties() const
{
    return properties_impl();
}

inline Generator::Task
VtsAtlasProvider::generateAtlas(const vts::TileId &tileId, Sink &sink
                                , const Sink::FileInfo &sfi
                                , bool atlas) const
{
    return generateAtlas_impl(tileId, sink, sfi, atlas);
}

} // namespace generator

#endif // mapproxy_generator_providers_hpp_included_
