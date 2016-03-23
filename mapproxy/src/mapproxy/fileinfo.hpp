#ifndef mapproxy_fileinfo_hpp_included_
#define mapproxy_fileinfo_hpp_included_

#include "vts-libs/storage/filetypes.hpp"
#include "vts-libs/storage/support.hpp"
#include "vts-libs/vts/basetypes.hpp"

#include "./resource.hpp"

namespace vs = vadstena::storage;
namespace vts = vadstena::vts;

struct FileInfo {
    FileInfo(const std::string &url);

    // full url
    std::string url;

    // reference frame
    std::string referenceFrame;

    // handling resource ID
    Resource::Id resourceId;

    // type of file
    enum class Type { unknown, file, tileFile, support };

    // type of file
    Type type;

    // tileset file, valid only when (type == Type::file)
    vs::File file;

    // tileset tile file, valid only when (type == Type::tileFile)
    vs::TileFile tileFile;

    // support file, valid only when (type == Type::support)
    const vs::SupportFile::Files::value_type *support;

    /** Request for raw file, not translation.
     */
    bool raw;

    /** tileId; valid only when (type == Type::tileFile)
     */
    vts::TileId tileId;

    /** Sub tile file. Used for textures in atlas.
     */
    unsigned int subTileFile;

    const vr::DataFile *registry;
};

#endif // mapproxy_fileinfo_hpp_included_
