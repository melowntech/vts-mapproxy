#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "utility/streams.hpp"

#include "vts-libs/vts/tileop.hpp"

#include "./error.hpp"
#include "./fileinfo.hpp"
#include "./browser2d.hpp"

namespace ba = boost::algorithm;

namespace constants {
    const std::string Config("mapConfig.json");
    const std::string Self("");
    const std::string Index("index.html");
}

FileInfo::FileInfo(const std::string &url)
    : url(url), type(Type::resourceFile)
{
    std::vector<std::string> components;
    ba::split(components, url, ba::is_any_of("/")
              , ba::token_compress_on);

    if (components.size() == 3) {
        // only reference frame -> allow only map config
        referenceFrame = components[1];
        if (components[2] != constants::Config) {
            LOGTHROW(err1, NotFound)
                << "URL <" << url << "> not found: reference frame "
                "supports only map configuration.";
        }

        type = Type::rfMapConfig;
        return;
    }

    if (components.size() != 6) {
        LOGTHROW(err1, NotFound)
            << "URL <" << url << "> not found: invalid number "
            "of path components.";
    }

    // full resource file path
    referenceFrame = components[1];
    generatorType = components[2];
    resourceId.group = components[3];
    resourceId.id = components[4];
    filename = components[5];
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
        } else {
            // another file -> parse as format
            type = Type::imagery;
            try {
                format = boost::lexical_cast<RasterFormat>(ext);
            } catch (boost::bad_lexical_cast) {
                return false;
            }
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
    case Type::imagery:
        return { contentType(format), lastModified };
    case Type::mask:
        return { contentType(MaskFormat), lastModified };
    case Type::support:
        return { support->contentType, support->lastModified };
    case Type::unknown:
        return {};
    }

    return {};
}
