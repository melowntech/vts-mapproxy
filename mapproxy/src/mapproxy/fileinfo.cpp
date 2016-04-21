#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/range/iterator.hpp>

#include "utility/streams.hpp"

#include "vts-libs/registry.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/support.hpp"

#include "./error.hpp"
#include "./fileinfo.hpp"
#include "./browser2d.hpp"

namespace ba = boost::algorithm;
namespace vr = vadstena::registry;

namespace constants {
    const std::string Config("mapConfig.json");
    const std::string Self("");
    const std::string Index("index.html");

    namespace tileset {
        const std::string Config("tileset.conf");
        const std::string Index("tileset.index");
    } // namespace tileset
} // namesapce constants

namespace {

template <typename E>
bool asEnum(const std::string &str, E &value)
{
    try {
        value = boost::lexical_cast<E>(str);
    } catch (boost::bad_lexical_cast) {
        return false;
    }
    return true;
}

template <typename E, typename Error>
void asEnumChecked(const std::string &str, E &value, const std::string message)
{
    if (!asEnum(str, value)) {
        LOGTHROW(err1, NotFound)
            << "Invalid value for enum <" << str << ">: " << message;
    }
}

const std::string& checkReferenceFrame(const std::string &referenceFrame)
{
    if (vr::Registry::referenceFrame(referenceFrame, std::nothrow)) {
        return referenceFrame;
    }

    LOGTHROW(err1, NotFound)
        << "<" << referenceFrame << "> is not known reference frame.";
    throw;
}

} // namespace

FileInfo::FileInfo(const std::string &url)
    : url(url), type(Type::resourceFile)
{
    auto end(url.end());
    auto qm(url.find('?'));
    if (qm != std::string::npos) {
        end = url.begin() + qm;
        query = url.substr(qm);
    }

    std::vector<std::string> components;
    {
        auto range(boost::make_iterator_range(url.begin(), end));
        ba::split(components, range, ba::is_any_of("/")
                  , ba::token_compress_on);
    }

    switch (components.size() - 1) {
    case 1:
        filename = components[1];

        if ((filename == constants::Index)
            || filename == constants::Self)
        {
            // /rf/index.html or /rf/ -> list types
            type = Type::referenceFrameListing;
            return;
        }
        // just /rf -> redir to /rf/
        type = Type::dirRedir;
        return;

    case 2:
        resourceId.referenceFrame = checkReferenceFrame(components[1]);
        filename = components[2];

        if (filename == constants::Config) {
            // /rf/mapConfig.json
            type = Type::referenceFrameMapConfig;
            return;
        } else if (filename == constants::Index) {
            // /rf/index.html -> browser
            type = Type::referenceFrameBrowser;
            return;
        } else if (filename == constants::Self) {
            // /rf/ -> list types
            type = Type::typeListing;
            return;
        }

        // just /rf/type -> redir to /rf/type/
        type = Type::dirRedir;
        return;

    case 3:
        // only reference frame -> allow only map config
        resourceId.referenceFrame = checkReferenceFrame(components[1]);
        asEnumChecked<Resource::Generator::Type, NotFound>
            (components[2], generatorType, "Unknown generator type.");
        filename = components[3];

        if (filename == constants::Config) {
            // /rf/type/mapConfig.json
            type = Type::typeMapConfig;
            return;
        } else if (filename == constants::Index) {
            // /rf/type/index.html -> browser
            type = Type::typeBrowser;
            return;
        } else if (filename == constants::Self) {
            // /rf/type/ -> list types
            type = Type::groupListing;
            return;
        }

        // just /rf/type/group -> redir to /rf/type/group/
        type = Type::dirRedir;
        return;

    case 4:
        // only reference frame -> allow only map config
        resourceId.referenceFrame = checkReferenceFrame(components[1]);
        asEnumChecked<Resource::Generator::Type, NotFound>
            (components[2], generatorType, "Unknown generator type.");
        resourceId.group = components[3];
        filename = components[4];

        if (filename == constants::Config) {
            // /rf/type/group/mapConfig.json
            type = Type::groupMapConfig;
            return;
        } else if (filename == constants::Index) {
            // /rf/type/group/index.html -> browser
            type = Type::groupBrowser;
            return;
        } else if (filename == constants::Self) {
            // /rf/type/group/ -> list ids
            type = Type::idListing;
            return;
        }

        // just /rf/type/group/id -> redir to /rf/type/group/id/
        type = Type::dirRedir;
        return;

    case 5:
        // full resource file path
        resourceId.referenceFrame = checkReferenceFrame(components[1]);
        asEnumChecked<Resource::Generator::Type, NotFound>
            (components[2], generatorType, "Unknown generator type.");
        resourceId.group = components[3];
        resourceId.id = components[4];
        filename = components[5];
        return;

    default:
        if (components.size() != 6) {
            LOGTHROW(err1, NotFound)
                << "URL <" << url << "> not found: invalid number "
                "of path components.";
        }
    }

}

namespace {

inline bool isDigit(char c) { return (c >= '0') && (c <= '9'); }

inline char positive(char c) { return c - '0'; }

template <unsigned int minWidth, char(*getter)(char), typename T>
inline const char* parsePartImpl(const char *p, T &value)
{
    bool prefix = false;
    char c(p[0]);
    switch (c) {
    case '-': case '+': return nullptr;
    case '0': prefix = true;
    }

    value = 0;

    const char *e(p);
    while (isDigit(c)) {
        value *= 10;
        value += getter(c);
        c = *++e;
    }

    auto dist(e - p);
    if (dist < minWidth) { return nullptr; }
    if (prefix && (dist > minWidth)) { return nullptr; }
    return e;
}

template <unsigned int minWidth, typename T>
inline const char* parsePart(const char *p, T &value)
{
    // only positive numbers are allowed
    return parsePartImpl<minWidth, positive>(p, value);
}

} // namespace

TmsFileInfo::TmsFileInfo(const FileInfo &fi, int flags)
    : fileInfo(fi), type(Type::unknown), support()
{
    if ([&]() -> bool
    {
        const char *p(fi.filename.c_str());

        if (!(p = parsePart<1>(p, tileId.lod))) { return false; }
        if (*p++ != '-') { return false; }

        if (!(p = parsePart<1>(p, tileId.x))) { return false; }
        if (*p++ != '-') { return false; }

        if (!(p = parsePart<1>(p, tileId.y))) { return false; }
        if (*p++ != '.') { return false; }

        std::string ext(p);
        if (ext == "mask") {
            // mask file
            type = Type::mask;
        } else if (ext == "meta") {
            // mask file
            type = Type::metatile;
        } else {
            // another file -> parse as format
            type = Type::image;
            if (!asEnum<RasterFormat>(ext, format)) { return false; }
        }

        return true;
    }()) {
        return;
    }

    if (constants::Config == fi.filename) {
        type = Type::config;
        return;
    }

    if (flags & FileFlags::browserEnabled) {
        LOG(debug) << "Browser enabled, checking browser files.";

        auto path(fi.filename);
        if (constants::Self == path) { path = constants::Index; }

        // support files
        auto fsupport(browser2d::supportFiles.find(path));
        if (fsupport != browser2d::supportFiles.end()) {
            type = Type::support;
            support = &fsupport->second;
            return;
        }
    } else {
        LOG(debug) << "Browser disabled, skipping browser files.";
    }
}

Sink::FileInfo TmsFileInfo::sinkFileInfo(std::time_t lastModified) const
{
    switch (type) {
    case Type::config:
        return { "application/json", lastModified };
    case Type::image:
        return { contentType(format), lastModified };
    case Type::mask:
        return { contentType(MaskFormat), lastModified };
    case Type::metatile:
        return { contentType(RasterMetatileFormat), lastModified };
    case Type::support:
        return { support->contentType, support->lastModified };
    case Type::unknown:
        return {};
    }

    return {};
}

SurfaceFileInfo::SurfaceFileInfo(const FileInfo &fi, int flags)
    : fileInfo(fi), type(Type::unknown), fileType(vs::File::config)
    , tileType(vts::TileFile::meta), raw(false), support(), registry()
{
    if (vts::fromFilename
        (tileId, tileType, subTileIndex, fi.filename, 0, &raw))
    {
        type = Type::tile;
        return;
    }

    // non-tile files
    if (constants::Config == fi.filename) {
        type = Type::file;
        fileType = vs::File::config;
        return;
    }

    if (constants::tileset::Config == fi.filename) {
        type = Type::file;
        fileType = vs::File::config;
        // this is raw file
        raw = true;
        return;
    }

    if (constants::tileset::Index == fi.filename) {
        type = Type::file;
        fileType = vs::File::tileIndex;
        return;
    }

    if (flags & FileFlags::browserEnabled) {
        LOG(debug) << "Browser enabled, checking browser files.";

        auto path(fi.filename);
        if (constants::Self == path) { path = constants::Index; }

        // support files
        auto fsupport(vts::supportFiles.find(path));
        if (fsupport != vts::supportFiles.end()) {
            type = Type::support;
            support = &fsupport->second;
            return;
        }
    } else {
        LOG(debug) << "Browser disabled, skipping browser files.";
    }

    // extra files, unknown to common machinery
    registry = vr::Registry::dataFile
        (fi.filename, vr::Registry::DataFileKey::filename, std::nothrow);
    if (registry) {
        type = Type::registry;
        return;
    }
}

Sink::FileInfo SurfaceFileInfo::sinkFileInfo(std::time_t lastModified) const
{
    switch (type) {
    case Type::file:
        switch (fileType) {
        case vts::File::config:
            return { "application/json", lastModified };

        case vts::File::tileIndex:
            return { "application/octet-stream", lastModified };

        default:
            return {};
        }
        break;

    case Type::tile:
        switch (tileType) {
        case vts::TileFile::meta:
            return { "application/octet-stream", lastModified };
        case vts::TileFile::mesh:
            return { "application/octet-stream", lastModified };
        case vts::TileFile::atlas:
            return { "image/jpeg", lastModified };
        case vts::TileFile::navtile:
            return { "image/jpeg", lastModified };
        }
        break;

    case Type::support:
        return { support->contentType, support->lastModified };

    case Type::registry:
        return { registry->contentType, lastModified };

    case Type::unknown:
        return {};
    }

    return {};
}
