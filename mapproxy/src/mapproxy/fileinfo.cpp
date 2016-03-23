#include "./fileinfo.hpp"

FileInfo::FileInfo(const std::string &url)
    : url(url)
{
}

#if 0
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
    vadstena::vts::TileId tileId;

    /** Sub tile file. Used for textures in atlas.
     */
    unsigned int subTileFile;

    const vr::DataFile *registry;
};
#endif
