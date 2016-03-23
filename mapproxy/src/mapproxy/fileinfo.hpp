#ifndef mapproxy_fileinfo_hpp_included_
#define mapproxy_fileinfo_hpp_included_

#include "./resource.hpp"

/** Parsed file information.
 */
struct FileInfo {
    FileInfo(const std::string &url);

    /**Full url.
     */
    std::string url;

    /** Reference frame.
     */
    std::string referenceFrame;

    /** Resource generator type.
     */
    std::string generatorType;

    /** Handling resource ID.
     */
    Resource::Id resourceId;

    /** Requested filename. Parsed by appropriate generator.
     */
    std::string filename;
};

#endif // mapproxy_fileinfo_hpp_included_
