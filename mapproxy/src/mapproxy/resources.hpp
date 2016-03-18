#ifndef mapproxy_resources_hpp_included_
#define mapproxy_resources_hpp_included_

#include "vts-libs/registry.hpp"

namespace vr = vadstena::registry;

struct Resource {
    enum class Type { raster, surface };
    enum class Driver { tms, dem, spereoid };

    std::string group;
    std::string id;
    vr::StringIdSet credits;

    /** URL path this resource is available at.
     */
    std::string path;

    // TODO: fill in other stuff here

    typedef std::map<std::string, Resource> Group;
    typedef std::map<std::string, Group> Groups;
};

#endif // mapproxy_resources_hpp_included_
