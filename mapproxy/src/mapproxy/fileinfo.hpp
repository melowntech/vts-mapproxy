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

#ifndef mapproxy_fileinfo_hpp_included_
#define mapproxy_fileinfo_hpp_included_

#include "geo/vectorformat.hpp"

#include "vts-libs/storage/support.hpp"
#include "vts-libs/vts/basetypes.hpp"
#include "vts-libs/vts/tileop.hpp"

#include "resource.hpp"
#include "sink.hpp"

namespace vs = vtslibs::storage;
namespace vts = vtslibs::vts;

namespace FileFlags { enum {
    none = 0x00
    , browserEnabled = 0x01
}; } // namesapce FileFlags

/** Parsed file information.
 */
struct FileInfo {
    FileInfo(const http::Request &request, int flags = FileFlags::none);

    /** Full url.
     */
    std::string url;

    /** Path component
     */
    std::string path;

    /** Query party of URL (?query)
     */
    std::string query;

    /** Delivery flags.
     */
    int flags;

    enum class Type {
        dirRedir
        , referenceFrameListing, typeListing, groupListing, idListing

        , referenceFrameBrowser, typeBrowser, groupBrowser

        , resourceFile
        , referenceFrameDems
    };

    /** Type of file to generate.
     */
    Type type;

    /** Resource generator type.
     *  Valid only if type >= Type::groupListing.
     */
    GeneratorInterface interface;

    /** Handling resource ID.
     *  Valid only if type >= Type::groupListing.
     */
    Resource::Id resourceId;

    /** Requested filename. Parsed by appropriate generator.
     *  Valid only if type == Type::resourceFile.
     */
    std::string filename;
};

/** Parsed TMS file information.
 */
struct TmsFileInfo {
    TmsFileInfo(const FileInfo &fi);

    Sink::FileInfo sinkFileInfo(std::time_t lastModified = -1) const;

    /** Parent information.
     */
    FileInfo fileInfo;

    enum class Type { unknown, config, definition, image
                      , mask, metatile, support };

    /** File type.
     */
    Type type;

    /** Valid only when type in (Type::image, Type::mask)
     */
    vts::TileId tileId;

    /** Valid only when type == Type::image
     */
    RasterFormat format;

    /** Valid only when type == Type::support;
     */
    const vs::SupportFile *support;
};

/** Parsed surface file information.
 */
struct SurfaceFileInfo {
    SurfaceFileInfo(const FileInfo &fi);

    Sink::FileInfo sinkFileInfo(std::time_t lastModified = -1) const;

    /** Parent information.
     */
    FileInfo fileInfo;

    enum class Type {
        unknown, file, tile, definition, support, registry, service
    };

    /** File type.
     */
    Type type;

    /** Valid only when type in Type::file
     */
    vts::File fileType;

    /** Valid only when type in Type::tile
     */
    vts::TileFile tileType;

    /** Valid only when type in Type::tile or Type::terrain
     */
    vts::TileId tileId;

    /** Valid only when type in Type::tile
     */
    unsigned int subTileIndex;

    /** Distinguishes non-regular file from interpreted (i.e. tileset.conf from
     *  mapConfig.json)
     */
    vts::FileFlavor flavor;

    /** Valid only when type == Type::support
     */
    const vs::SupportFile *support;

    /** Valid only when type == Type::registry
     */
    const vr::DataFile *registry;

    /** Valid only when type == Type::service
     */
    unsigned int serviceFile;
};

/** Parsed surface file information.
 */
struct GeodataFileInfo {
    GeodataFileInfo(const FileInfo &fi, bool tiled, geo::VectorFormat format);

    Sink::FileInfo sinkFileInfo(std::time_t lastModified = -1) const;

    /** Parent information.
     */
    FileInfo fileInfo;

    enum class Type {
        unknown, config, definition, geo, metatile, support, registry, style
    };

    /** File type.
     */
    Type type;

    /** Valid only when type in (Type::geo, Type::metatile)
     */
    vts::TileId tileId;

    /** Valid only when type == Type::support;
     */
    const vs::SupportFile *support;

    /** Valid only when type == Type::registry;
     */
    const vr::DataFile *registry;

    /** File format. Passed in ctor.
     */
    geo::VectorFormat format;
};

/** Parsed terrain file information.
 */
struct TerrainFileInfo {
    TerrainFileInfo(const FileInfo &fi);

    Sink::FileInfo sinkFileInfo(std::time_t lastModified = -1) const;

    /** Parent information.
     */
    FileInfo fileInfo;

    enum class Type {
        unknown, tile, definition, support, cesiumConf
    };

    /** File type.
     */
    Type type;

    /** Valid only when type == Type::terrain
     */
    vts::TileId tileId;

    /** Valid only when type == Type::support
     */
    const vs::SupportFile *support;
};

/** Parsed TMS file information.
 */
struct WmtsFileInfo {
    WmtsFileInfo(const FileInfo &fi);

    Sink::FileInfo sinkFileInfo(std::time_t lastModified = -1) const;

    /** Parent information.
     */
    FileInfo fileInfo;

    enum class Type { unknown, capabilities, support };

    /** File type.
     */
    Type type;

    /** Valid only when type == Type::support;
     */
    const vs::SupportFile *support;
};

#endif // mapproxy_fileinfo_hpp_included_
