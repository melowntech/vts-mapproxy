/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/lexical_cast.hpp>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/raise.hpp"

#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/registry/py.hpp"

#include "./error.hpp"
#include "./resource.hpp"
#include "./generator.hpp"
#include "./support/python.hpp"

namespace fs = boost::filesystem;
namespace python = boost::python;

namespace vr = vtslibs::registry;
namespace vs = vtslibs::storage;

namespace detail {

void parseCredits(Resource &r, const python::object &object
                  , const char *name)
{
    const auto value(object[name]);

    for (python::stl_input_iterator<python::object> ivalue(value), evalue;
         ivalue != evalue; ++ivalue)
    {
        const auto element(*ivalue);

        const auto &credit([&]() -> const vr::Credit&
        {
            python::extract<int> asInt(element);
            if (asInt.check()) {
                int key(asInt);
                const auto credit(r.registry.credits(key, std::nothrow));
                return (credit ? *credit : vr::system.credits(key));
            }

            auto key(py2utf8(*ivalue));
            const auto credit(r.registry.credits(key, std::nothrow));
            return (credit ? *credit : vr::system.credits(key));
        }());

        r.credits.insert(DualId(credit.id, credit.numericId));
    }
}

void parseDefinition(Resource &r, const python::dict &value)
{
    auto definition(Generator::definition(r.generator));
    definition->from(value);
    r.definition(definition);
}

boost::optional<Resource::Id> parseResourceId(const python::dict &value)
{
    try {
        Resource::Id id;
        id.referenceFrame = "*";
        id.group = py2utf8(value["group"]);
        id.id = py2utf8(value["id"]);
        return id;
    } catch (const std::exception &e) {
        LOG(info2) << "Failed to parse resource ID from python object "
                   << ": <" << e.what() << ">.";
    }
    return boost::none;
}

FileClassSettings parseFileClassSettings(const python::dict &value
                                         , const FileClassSettings &defaults)
{
    FileClassSettings fcs(defaults);

    for (python::stl_input_iterator<python::str> ivalue(value), evalue;
         ivalue != evalue; ++ivalue)
    {
        auto fc(boost::lexical_cast<FileClass>(py2utf8(*ivalue)));
        auto maxAge(python::extract<int>(value[*ivalue])());
        fcs.setMaxAge(fc, maxAge);
    }
    return fcs;
}

Resource::list parseResource(const Resource::Id &id, const python::dict &value
                             , const FileClassSettings &fileClassSettings)
{
    Resource r(value.has_key("maxAge")
               ? parseFileClassSettings(python::dict(value["maxAge"])
                                        , fileClassSettings)
               : fileClassSettings);
    r.id = id;

    std::string tmp(py2utf8(value["type"]));
    r.generator.type = boost::lexical_cast<Resource::Generator::Type>(tmp);
    r.generator.driver = py2utf8(value["driver"]);
    r.comment = py2utf8(value["comment"]);

    if (value.has_key("registry")) {
        fromPython(r.registry, python::dict(value["registry"]));
    }

    parseCredits(r, value, "credits");

    const auto referenceFrames(value["referenceFrames"]);

    parseDefinition(r, python::dict(value["definition"]));

    Resource::list out;

    for (python::stl_input_iterator<python::str>
             ireferenceFrames(referenceFrames), ereferenceFrames;
         ireferenceFrames != ereferenceFrames; ++ireferenceFrames)
    {
        const auto name(py2utf8(*ireferenceFrames));
        const auto content(referenceFrames[*ireferenceFrames]);

        out.push_back(r);
        auto &rr(out.back());
        rr.id.referenceFrame = name;

        // NB: function either returns valid reference of throws
        rr.referenceFrame = &vr::system.referenceFrames(name);

        const auto lodRange(content["lodRange"]);
        rr.lodRange.min = python::extract<vts::Lod>(lodRange[0]);
        rr.lodRange.max = python::extract<vts::Lod>(lodRange[1]);

        const auto tileRange(content["tileRange"]);
        const auto tileRange_min(tileRange[0]);
        rr.tileRange.ll(0) = python::extract<unsigned int>(tileRange_min[0]);
        rr.tileRange.ll(1) = python::extract<unsigned int>(tileRange_min[1]);

        const auto tileRange_max(tileRange[1]);
        rr.tileRange.ur(0) = python::extract<unsigned int>(tileRange_max[0]);
        rr.tileRange.ur(1) = python::extract<unsigned int>(tileRange_max[1]);
    }

    return out;
}

void parseResources(Resource::map &resources, const python::list &value
                    , ResourceLoadErrorCallback error
                    , const FileClassSettings &fileClassSettings)
{
    // process all definitions
    for (python::stl_input_iterator<python::dict> ivalue(value), evalue;
         ivalue != evalue; ++ivalue)
    {
        python::dict dict(*ivalue);
        auto id(parseResourceId(dict));
        if (!id) { continue; }

        try {
            auto resList(parseResource(*id, dict, fileClassSettings));
            for (const auto &res : resList) {
                if (!resources.insert(Resource::map::value_type(res.id, res))
                    .second)
                {
                    auto message(utility::format
                                 ("Duplicate entry for <%s>.", res.id));
                    LOG(warn1) << message;
                    if (error) { error(res.id, message); }
                }
            }
        } catch (const std::exception &e) {
            LOG(warn1) << "Failed to parse resource "
                       << *id << ": <" << e.what() << ">.";
            if (error) { error(*id, e.what()); }
        }

    }
}

} // namespace detail

Resource::map
loadResourcesFromPython(const boost::any &pylist
                        , ResourceLoadErrorCallback error
                        , const FileClassSettings &fileClassSettings)
{
    const auto &list(boost::any_cast<const python::list&>(pylist));

    Resource::map resources;
    detail::parseResources(resources, list, error, fileClassSettings);
    return resources;
}
