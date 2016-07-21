#include <boost/lexical_cast.hpp>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/raise.hpp"

#include "vts-libs/vts/tileop.hpp"

#include "./error.hpp"
#include "./resource.hpp"
#include "./generator.hpp"
#include "./support/python.hpp"

namespace fs = boost::filesystem;
namespace python = boost::python;

namespace vr = vadstena::registry;
namespace vs = vadstena::storage;

namespace detail {

void parseCredits(DualId::set &ids, const python::object &object
                  , const char *name)
{
    const auto &value(object[name]);

    for (python::stl_input_iterator<python::object> ivalue(value), evalue;
         ivalue != evalue; ++ivalue)
    {
        const auto &element(*ivalue);

        const auto &credit([&]() -> const vr::Credit&
        {
            python::extract<int> asInt(element);
            if (asInt.check()) {
                return vr::system.credits(asInt);
            }

            return vr::system.credits(py2utf8(element));
        }());

        ids.insert(DualId(credit.id, credit.numericId));
    }
}

void parseDefinition(Resource &r, const python::dict &value)
{
    auto definition(Generator::definition(r.generator));
    definition->from(value);
    r.definition(definition);
}

Resource::list parseResource(const python::object &value)
{
    Resource r;
    r.id.group = py2utf8(value["group"]);
    r.id.id = py2utf8(value["id"]);

    std::string tmp(py2utf8(value["type"]));
    r.generator.type = boost::lexical_cast<Resource::Generator::Type>(tmp);
    r.generator.driver = py2utf8(value["driver"]);

    parseCredits(r.credits, value, "credits");

    const auto &referenceFrames(value["referenceFrames"]);

    parseDefinition(r, python::dict(value["definition"]));

    Resource::list out;

    for (python::stl_input_iterator<python::str>
             ireferenceFrames(referenceFrames), ereferenceFrames;
         ireferenceFrames != ereferenceFrames; ++ireferenceFrames)
    {
        const auto &name(py2utf8(*ireferenceFrames));
        const auto &content(referenceFrames[*ireferenceFrames]);

        out.push_back(r);
        auto &rr(out.back());
        rr.id.referenceFrame = name;

        // NB: function either returns valid reference of throws
        rr.referenceFrame = &vr::system.referenceFrames(name);

        const auto &lodRange(content["lodRange"]);
        rr.lodRange.min = python::extract<vts::Lod>(lodRange[0]);
        rr.lodRange.max = python::extract<vts::Lod>(lodRange[1]);

        const auto &tileRange(content["tileRange"]);
        const auto &tileRange_min(tileRange[0]);
        rr.tileRange.ll(0) = python::extract<unsigned int>(tileRange_min[0]);
        rr.tileRange.ll(1) = python::extract<unsigned int>(tileRange_min[1]);

        const auto &tileRange_max(tileRange[1]);
        rr.tileRange.ur(0) = python::extract<unsigned int>(tileRange_max[0]);
        rr.tileRange.ur(1) = python::extract<unsigned int>(tileRange_max[1]);
    }

    return out;
}

void parseResources(Resource::map &resources, const python::list &value)
{
    // process all definitions
    for (python::stl_input_iterator<python::dict> ivalue(value), evalue;
         ivalue != evalue; ++ivalue)
    {
        auto resList(parseResource(python::dict(*ivalue)));

        for (const auto &res : resList) {
            if (!resources.insert(Resource::map::value_type(res.id, res))
                .second)
            {
                LOGTHROW(err1, Error)
                    << "Duplicate entry for <" << res.id << ">.";
            }
        }
    }
}

} // namespace detail

Resource::map loadResourcesFromPython(const boost::any &pylist)
{
    const auto &list(boost::any_cast<const python::list&>(pylist));

    Resource::map resources;
    detail::parseResources(resources, list);
    return resources;
}
