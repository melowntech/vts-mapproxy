#ifndef mapproxy_fileinfo_hpp_included_
#define mapproxy_fileinfo_hpp_included_

#include "./resource.hpp"

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

#endif // mapproxy_fileinfo_hpp_included_
