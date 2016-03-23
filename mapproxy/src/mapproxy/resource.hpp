#ifndef mapproxy_resource_hpp_included_
#define mapproxy_resource_hpp_included_

#include <iostream>

#include <boost/filesystem/path.hpp>
#include <boost/any.hpp>

#include "utility/enum-io.hpp"

#include "vts-libs/registry.hpp"

namespace vr = vadstena::registry;

struct Resource {
    struct Id {
        std::string group;
        std::string id;

        Id() {}
        Id(const std::string &group, const std::string &id)
            : group(group), id(id) {}
        bool operator<(const Id &o) const;
        bool operator==(const Id &o) const;
    };

    struct Generator {
        std::string type;
        std::string driver;

        Generator() {}
        Generator(const std::string &type, const std::string &driver)
            : type(type), driver(driver) {}
        bool operator<(const Generator &o) const;
        bool operator==(const Generator &o) const;
    };

    Id id;
    Generator generator;

    vr::IdSet credits;

    struct ReferenceFrame {
        const vr::ReferenceFrame *referenceFrame;
        vr::LodRange lodRange;
        vr::TileRange tileRange;

        typedef std::map<std::string, ReferenceFrame> map;
        bool operator==(const ReferenceFrame &o) const;
    };

    ReferenceFrame::map referenceFrames;

    /** Definition: based on type and driver, created by resource
     *  parser/generator and interpreted by driver.
     */
    boost::any definition;

    typedef std::map<Id, Resource> map;
    typedef std::vector<Resource> list;

    Resource() {}

    bool operator==(const Resource &o) const;
    bool operator!=(const Resource &o) const;
};

UTILITY_GENERATE_ENUM(RasterFormat,
    ((jpg))
    ((png))
)

namespace resdef {

struct TmsRaster {
    static Resource::Generator generator;

    boost::filesystem::path datasetPath;
    boost::filesystem::path maskPath;
    RasterFormat format;

    TmsRaster() : format(RasterFormat::jpg) {}
    bool operator==(const TmsRaster &o) const;
};

struct SurfaceSpheroid {
    static Resource::Generator generator;

    double a;
    double b;
    unsigned int textureLayerId;

    SurfaceSpheroid() : a(), b(), textureLayerId() {}
    bool operator==(const SurfaceSpheroid &o) const;
};

struct SurfaceDem {
    static Resource::Generator generator;

    boost::filesystem::path datasetPath;
    unsigned int textureLayerId;

    SurfaceDem() : textureLayerId() {}
    bool operator==(const SurfaceDem &o) const;
};

} // namespace resdef

/** Load resources from given path.
 */
Resource::map loadResources(const boost::filesystem::path &path);

/** Load single resource from given path.
 */
Resource loadResource(const boost::filesystem::path &path);

/** Save single resource to given path.
 */
void save(const boost::filesystem::path &path, const Resource &resource);

// inlines + IO

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Resource::Id &rid)
{
    return os << rid.group << '/' << rid.id;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const Resource::Generator &g)
{
    return os << g.type << '/' << g.driver;
}

inline bool Resource::Id::operator<(const Id &o) const {
    if (group < o.group) { return true; }
    else if (o.group < group) { return true; }
    return id < o.id;
}

inline bool Resource::Id::operator==(const Id &o) const {
    return ((group == o.group) || (id == o.id));
}

inline bool Resource::Generator::operator<(const Generator &o) const {
    if (type < o.type) { return true; }
    else if (o.type < type) { return true; }
    return driver < o.driver;
}

inline bool Resource::Generator::operator==(const Generator &o) const {
    return ((type == o.type) || (driver == o.driver));
}

inline bool Resource::operator!=(const Resource &o) const
{
    return !(*this == o);
}
#endif // mapproxy_resource_hpp_included_
