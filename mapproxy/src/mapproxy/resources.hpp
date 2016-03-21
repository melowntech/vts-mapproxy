#ifndef mapproxy_resources_hpp_included_
#define mapproxy_resources_hpp_included_

#include "vts-libs/registry.hpp"

namespace vr = vadstena::registry;

struct Resource {
    std::string group;
    std::string id;
    std::string type;

    vr::IdSet credits;

    /** URL path this resource is available at.
     */
    std::string pathTemplate;

    struct ReferenceFrame {
        const vr::ReferenceFrame *referenceFrame;
        vr::LodRange lodRange;
        vr::TileRange tileRange;

        // TODO: add type/driver
        // TODO: definition

        /** Definition: based on type and driver, created by resource
         *  parser/generator and interpreted by driver.
         */
        boost::any definition;

        typedef std::map<std::string, ReferenceFrame> map;
    };

    ReferenceFrame::map referenceFrames;

    // TODO: fill in other stuff here

    typedef std::map<std::string, Resource> Group;
    typedef std::map<std::string, Group> Groups;

    Resource() {}
};

#endif // mapproxy_resources_hpp_included_
