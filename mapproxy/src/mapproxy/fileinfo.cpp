#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "utility/streams.hpp"

#include "vts-libs/vts/tileop.hpp"

#include "./error.hpp"
#include "./fileinfo.hpp"

namespace ba = boost::algorithm;

namespace constants {
    const std::string Config("mapConfig.json");
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
