#ifndef mapproxy_fileinfo_hpp_included_
#define mapproxy_fileinfo_hpp_included_

#include "vts-libs/storage/support.hpp"
#include "vts-libs/vts/basetypes.hpp"
#include "vts-libs/vts/tileop.hpp"

#include "./resource.hpp"
#include "./sink.hpp"

namespace vs = vadstena::storage;
namespace vts = vadstena::vts;

namespace FileFlags { enum {
    none = 0x00
    , browserEnabled = 0x01
}; } // namesapce FileFlags

/** Parsed file information.
 */
struct FileInfo {
    FileInfo(const std::string &url);

    /** Full url.
     */
    std::string url;

    enum class Type {
        dirRedir
        , referenceFrameListing, typeListing, groupListing, idListing

        , referenceFrameBrowser, typeBrowser, groupBrowser

        , referenceFrameMapConfig, typeMapConfig, groupMapConfig
        , resourceFile
    };

    /** Type of file to generate.
     */
    Type type;

    /** Resource generator type.
     *  Valid only if type == Type::resourceFile.
     */
    Resource::Generator::Type generatorType;

    /** Handling resource ID.
     *  Valid only if type == Type::resourceFile.
     */
    Resource::Id resourceId;

    /** Requested filename. Parsed by appropriate generator.
     *  Valid only if type == Type::resourceFile.
     */
    std::string filename;

    /** Query party of URL (?query)
     */
    std::string query;
};

/** Parsed TMS file information.
 */
struct TmsFileInfo {
    TmsFileInfo(const FileInfo &fi, int flags = FileFlags::none);

    Sink::FileInfo sinkFileInfo(std::time_t lastModified = -1) const;

    /** Parent information.
     */
    FileInfo fileInfo;

    enum class Type { unknown, config, definition, image
                      , mask, metatile, support };

    /** File type.
     */
    Type type;

    /** Valid only when type in (Type::tile, Type::mask)
     */
    vts::TileId tileId;

    /** Valid only when type == Type::tile;
     */
    RasterFormat format;

    /** Valid only when type == Type::support;
     */
    const vs::SupportFile *support;
};

/** Parsed surface file information.
 */
struct SurfaceFileInfo {
    SurfaceFileInfo(const FileInfo &fi, int flags = FileFlags::none);

    Sink::FileInfo sinkFileInfo(std::time_t lastModified = -1) const;

    /** Parent information.
     */
    FileInfo fileInfo;

    enum class Type { unknown, file, tile, definition, support, registry };

    /** File type.
     */
    Type type;

    /** Valid only when type in Type::file
     */
    vts::File fileType;

    /** Valid only when type in Type::tile
     */
    vts::TileFile tileType;

    /** Valid only when type in Type::tile
     */
    vts::TileId tileId;

    /** Valid only when type in Type::tile
     */
    unsigned int subTileIndex;

    /** Distinguishes raw file from interpreted (i.e. tileset.conf from
     *  mapConfig.json)
     */
    bool raw;

    /** Valid only when type == Type::support;
     */
    const vs::SupportFile *support;

    /** Valid only when type == Type::registry;
     */
    const vr::DataFile *registry;
};

#endif // mapproxy_fileinfo_hpp_included_
