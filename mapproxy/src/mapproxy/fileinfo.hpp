#ifndef mapproxy_fileinfo_hpp_included_
#define mapproxy_fileinfo_hpp_included_

#include "utility/enum-io.hpp"

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

    // resource generator type
    std::string generatorType;

    // handling resource ID
    Resource::Id resourceId;

    /** Requested filename. Parsed by appropriate generator.
     */
    std::string filename;
};

#endif // mapproxy_fileinfo_hpp_included_
