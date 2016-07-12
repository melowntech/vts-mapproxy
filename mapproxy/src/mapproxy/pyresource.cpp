#include <boost/lexical_cast.hpp>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/raise.hpp"

#include "vts-libs/vts/tileop.hpp"

#include "./error.hpp"
#include "./resource.hpp"

namespace fs = boost::filesystem;
namespace python = boost::python;

namespace vr = vadstena::registry;
namespace vs = vadstena::storage;

namespace detail {

std::string utf8(boost::python::object s)
{
    using namespace python;
    return extract<std::string>(s.attr("encode")("utf-8"));
}

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
                return vr::Registry::credit(asInt);
            }

            return vr::Registry::credit(utf8(element));
        }());

        ids.insert(DualId(credit.id, credit.numericId));
    }
}

void parseDefinition(resdef::TmsRaster &def, const python::dict &value)
{
    def.dataset = utf8(value["dataset"]);

    if (value.has_key("mask")) {
        def.mask = utf8(value["mask"]);
    }

    if (value.has_key("format")) {
        try {
            def.format = boost::lexical_cast<RasterFormat>
                (utf8(value["format"]));
        } catch (boost::bad_lexical_cast) {
            utility::raise<Error>
                ("Value stored in format is not RasterFormat value");
        }
    }
}

void parseDefinition(resdef::TmsRasterRemote &def, const python::dict &value)
{
    def.remoteUrl = utf8(value["remoteUrl"]);

    if (value.has_key("mask")) {
        def.mask = utf8(value["mask"]);
    }
}

void parseDefinition(resdef::SurfaceSpheroid &def, const python::dict &value)
{
    if (value.has_key("textureLayerId")) {
        def.textureLayerId = python::extract<int>(value["textureLayerId"]);
    }

    if (value.has_key("geoidGrid")) {
        def.geoidGrid = utf8(value["geoidGrid"]);
    }
}

void parseDefinition(resdef::SurfaceDem &def, const python::dict &value)
{
    def.dataset = utf8(value["dataset"]);

    if (value.has_key("mask")) {
        def.mask = utf8(value["mask"]);
    }

    if (value.has_key("textureLayerId")) {
        def.textureLayerId = python::extract<int>(value["textureLayerId"]);
    }

    if (value.has_key("geoidGrid")) {
        def.geoidGrid = utf8(value["geoidGrid"]);
    }
}

void parseDefinition(Resource &r, const python::dict &value)
{
    if (r.generator == resdef::TmsRaster::generator) {
        return parseDefinition
            (r.assignDefinition<resdef::TmsRaster>(), value);
    }
    if (r.generator == resdef::TmsRasterRemote::generator) {
        return parseDefinition
            (r.assignDefinition<resdef::TmsRasterRemote>(), value);
    }
    if (r.generator == resdef::SurfaceSpheroid::generator) {
        return parseDefinition
            (r.assignDefinition<resdef::SurfaceSpheroid>(), value);
    }
    if (r.generator == resdef::SurfaceDem::generator) {
        return parseDefinition
            (r.assignDefinition<resdef::SurfaceDem>(), value);
    }

    LOGTHROW(err1, UnknownGenerator)
        << "Unknown generator <" << r.generator << ">.";
}

Resource::list parseResource(const python::object &value)
{
    Resource r;
    r.id.group = utf8(value["group"]);
    r.id.id = utf8(value["id"]);

    std::string tmp(utf8(value["type"]));
    r.generator.type = boost::lexical_cast<Resource::Generator::Type>(tmp);
    r.generator.driver = utf8(value["driver"]);

    parseCredits(r.credits, value, "credits");

    const auto &referenceFrames(value["referenceFrames"]);

    parseDefinition(r, python::dict(value["definition"]));

    Resource::list out;

    for (python::stl_input_iterator<python::str>
             ireferenceFrames(referenceFrames), ereferenceFrames;
         ireferenceFrames != ereferenceFrames; ++ireferenceFrames)
    {
        const auto &name(utf8(*ireferenceFrames));
        const auto &content(referenceFrames[*ireferenceFrames]);

        out.push_back(r);
        auto &rr(out.back());
        rr.id.referenceFrame = name;

        // NB: function either returns valid reference of throws
        rr.referenceFrame = &vr::Registry::referenceFrame(name);

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
