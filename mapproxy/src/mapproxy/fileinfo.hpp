#ifndef mapproxy_fileinfo_hpp_included_
#define mapproxy_fileinfo_hpp_included_

#include "vts-libs/storage/support.hpp"
#include "vts-libs/vts/basetypes.hpp"

#include "./resource.hpp"
#include "./contentgenerator.hpp"

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

    /** Reference frame.
     */
    std::string referenceFrame;

    enum class Type { rfMapConfig, resourceFile };

    /** Type of file to generate.
     */
    Type type;

    /** Resource generator type.
     *  Valid only if type == Type::resourceFile.
     */
    std::string generatorType;

    /** Handling resource ID.
     *  Valid only if type == Type::resourceFile.
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
    TmsFileInfo(const FileInfo &fi, int flags = FileFlags::none);

    Sink::FileInfo sinkFileInfo(std::time_t lastModified = -1) const;

    /** Parent information.
     */
    FileInfo fileInfo;

    enum class Type { config, imagery, mask, support };

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

#endif // mapproxy_fileinfo_hpp_included_
