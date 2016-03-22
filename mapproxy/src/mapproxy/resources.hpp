#ifndef mapproxy_resources_hpp_included_
#define mapproxy_resources_hpp_included_

#include <iostream>

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
    };

    struct Generator {
        std::string type;
        std::string driver;

        Generator() {}
        Generator(const std::string &type, const std::string &driver)
            : type(type), driver(driver) {}
        bool operator<(const Generator &o) const;
    };

    Id id;
    Generator generator;

    vr::IdSet credits;

    /** URL path this resource is available at.
     */
    std::string pathTemplate;

    struct ReferenceFrame {
        const vr::ReferenceFrame *referenceFrame;
        vr::LodRange lodRange;
        vr::TileRange tileRange;

        typedef std::map<std::string, ReferenceFrame> map;
    };

    ReferenceFrame::map referenceFrames;

    /** Definition: based on type and driver, created by resource
     *  parser/generator and interpreted by driver.
     */
    boost::any definition;

    // TODO: fill in other stuff here

    typedef std::map<Id, Resource> map;
    typedef std::vector<Resource> list;

    Resource() {}
};

// inlines

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

inline bool Resource::Generator::operator<(const Generator &o) const {
    if (type < o.type) { return true; }
    else if (o.type < type) { return true; }
    return driver < o.driver;
}

#endif // mapproxy_resources_hpp_included_
