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

#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "vts-libs/registry/json.hpp"
#include "vts-libs/vts/tileop.hpp"

#include "./support/glob.hpp"

#include "./error.hpp"
#include "./resource.hpp"
#include "./generator.hpp"

namespace fs = boost::filesystem;

namespace vr = vtslibs::registry;
namespace vs = vtslibs::storage;

namespace detail {

void parseCredits(Resource &r, const Json::Value &object
                  , const char *name)
{
    const Json::Value &value(object[name]);

    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of " << name << " is not an array.";
    }

    for (const auto &element : value) {
        const auto credit([&]() -> const vr::Credit&
        {
            if (element.isIntegral()) {
                auto value(element.asInt());
                const auto credit(r.registry.credits(value, std::nothrow));
                return (credit ? *credit : vr::system.credits(value));
            }

            Json::check(element, Json::stringValue);
            auto value(element.asString());
            const auto credit(r.registry.credits(value, std::nothrow));
            return (credit ? *credit : vr::system.credits(value));
        }());

        r.credits.insert(DualId(credit.id, credit.numericId));
    }
}

void parseDefinition(Resource &r, const Json::Value &value)
{
    auto definition(Generator::definition(r.generator));
    definition->from(value);
    r.definition(definition);
}

Json::Value buildDefinition(const Resource &r)
{
    boost::any tmp(Json::Value(Json::objectValue));
    r.definition()->to(tmp);
    return boost::any_cast<const Json::Value&>(tmp);
}

FileClassSettings parseFileClassSettings(const Json::Value &value
                                         , const FileClassSettings &defaults)
{
    if (value.isNull()) { return defaults; }

    FileClassSettings fcs(defaults);
    for (const auto &name : value.getMemberNames()) {
        auto fc(boost::lexical_cast<FileClass>(name));
        long maxAge;
        Json::get(maxAge, value, name.c_str());
        fcs.setMaxAge(fc, maxAge);
    }
    return fcs;
}

Resource::list parseResource(const Json::Value &value
                             , const FileClassSettings &fileClassSettings)
{
    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Resource definition is not an object.";
    }

    Resource r(parseFileClassSettings(value["maxAge"], fileClassSettings));

    Json::get(r.id.group, value, "group");
    Json::get(r.id.id, value, "id");
    std::string tmp;
    Json::get(tmp, value, "type");
    r.generator.type = boost::lexical_cast<Resource::Generator::Type>(tmp);
    Json::get(r.generator.driver, value, "driver");
    if (value.isMember("comment")) {
        Json::get(r.comment, value, "comment");
    }
    if (value.isMember("revision")) {
        Json::get(r.revision, value, "revision");
    }

    if (value.isMember("registry")) {
        fromJson(r.registry, value["registry"]);
    }
    parseCredits(r, value, "credits");

    const Json::Value &referenceFrames(value["referenceFrames"]);
    if (!referenceFrames.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of referenceFrames is not an object.";
    }

    parseDefinition(r, value["definition"]);

    Resource::list out;

    for (const auto &name : referenceFrames.getMemberNames()) {
        const auto &content(referenceFrames[name]);

        out.push_back(r);
        auto &rr(out.back());
        rr.id.referenceFrame = name;

        // NB: function either returns valid reference of throws
        rr.referenceFrame = &vr::system.referenceFrames(name);

        Json::get(rr.lodRange.min, content, "lodRange", 0);
        Json::get(rr.lodRange.max, content, "lodRange", 1);
        rr.tileRange = vr::tileRangeFromJson(content["tileRange"]);
    }

    return out;
}

void parseResources(Resource::map &resources, const Json::Value &value
                    , const FileClassSettings &fileClassSettings
                    , const fs::path &path)
{
    const auto dir(path.parent_path());

    // load part of include
    const auto includeLoad([&](const fs::path &includePath) -> void
    {
        LOG(info2) << "Loading resources from file " << includePath
                   << " included from " << path << ".";

        // open stream
        std::ifstream f;
        f.exceptions(std::ios::badbit | std::ios::failbit);

        try {
            f.open(includePath.string(), std::ios_base::in);
        } catch (const std::exception &e) {
            LOGTHROW(err1, IOError)
                << "Unable to load resources " << includePath
                << ": <" << e.what() << ">.";
        }

        // load data
        const auto config
            (Json::read<FormatError>(f, includePath, "resources"));

        // and recurse
        try {
            parseResources(resources, config, fileClassSettings
                           , includePath);
        } catch (const Json::Error &e) {
            LOGTHROW(err1, FormatError)
                << "Invalid resource config file " << includePath
                << " format: <" << e.what() << ">.";
        } catch (const vs::Error &e) {
            LOGTHROW(err1, FormatError)
                << "Invalid resource config file " << includePath
                << " format: <" << e.what() << ">.";
        }
    });

    // handle include of glob-expanded patter
    const auto include([&](const fs::path &value) -> void
    {
        const auto includePath(fs::absolute(value, dir));

        auto paths(globPath(includePath));
        for (const auto &path : paths) {
            // ignore directories
            if (path.filename() == ".") { continue; }
            includeLoad(path);
        }
    });

    // distribute include hased on JSON value
    const auto includeJson([&](const Json::Value &value) -> void
    {
        if (value.type() == Json::stringValue) {
            return include(value.asString());
        }

        LOGTHROW(err1, Json::Error)
            << "Include declaration must be a string or an array of strings.";
    });

    const auto processDefinition([&](const Json::Value &item) -> void
    {
        if (!item.isObject()) {
            LOGTHROW(err1, Json::Error)
                << "Resource definition is not an object.";
        }

        // check for special resources
        if (item.isMember("include")) {
            const auto &jinclude(item["include"]);
            if (jinclude.type() == Json::arrayValue) {
                for (const auto &element : jinclude) {
                    includeJson(element);
                }
            } else {
                includeJson(jinclude);
            }
            return;
        }

        // parse resource and remember
        auto resList(parseResource(item, fileClassSettings));

        for (const auto &res : resList) {
            if (!resources.insert(Resource::map::value_type(res.id, res))
                .second)
            {
                LOGTHROW(err1, Json::Error)
                    << "Duplicate entry for <" << res.id << ">.";
            }
        }
    });

    switch (value.type()) {
    case Json::arrayValue:
        // process all definitions
        for (const auto &item : value) {
            processDefinition(item);
        }
        break;

    case Json::objectValue:
        // single definition
        processDefinition(value);
        break;

    default:
        LOGTHROW(err1, Json::Error)
            << path << ": Type of top-level configuration is "
            "not an array nor an object.";
    };
}

Resource::map loadResources(std::istream &in, const fs::path &path
                            , const FileClassSettings &fileClassSettings)
{
    auto config(Json::read<FormatError>(in, path, "resources"));

    Resource::map resources;

    try {
        parseResources(resources, config, fileClassSettings, path);
    } catch (const Json::Error &e) {
        LOGTHROW(err1, FormatError)
            << "Invalid resource config file " << path
            << " format: <" << e.what() << ">.";
    } catch (const vs::Error &e) {
        LOGTHROW(err1, FormatError)
            << "Invalid resource config file " << path
            << " format: <" << e.what() << ">.";
    }

    return resources;
}

Resource::list loadResource(std::istream &in, const fs::path &path
                            , const FileClassSettings &fileClassSettings)
{
    auto config(Json::read<FormatError>(in, path, "resource"));

    try {
        return parseResource(config, fileClassSettings);
    } catch (const Json::Error &e) {
        LOGTHROW(err1, FormatError)
            << "Invalid resource config file " << path
            << " format: <" << e.what() << ">.";
    } catch (const vs::Error &e) {
        LOGTHROW(err1, FormatError)
            << "Invalid resource config file " << path
            << " format: <" << e.what() << ">.";
    }

    throw;
}

void buildResource(Json::Value &value, const Resource &r)
{
    value["group"] = r.id.group;
    value["id"] = r.id.id;
    value["type"] = boost::lexical_cast<std::string>(r.generator.type);
    value["driver"] = r.generator.driver;
    value["comment"] = r.comment;
    value["revision"] = r.revision;

    value["registry"] = vr::asJson(r.registry);

    auto &credits(value["credits"] = Json::arrayValue);
    for (auto cid : r.credits) { credits.append(cid.id); }

    Json::Value &referenceFrames(value["referenceFrames"]);
    auto &content(referenceFrames[r.id.referenceFrame] = Json::objectValue);

    auto &lodRange(content["lodRange"] = Json::arrayValue);
    lodRange.append(r.lodRange.min);
    lodRange.append(r.lodRange.max);

    auto &tileRange(content["tileRange"] = Json::arrayValue);
    auto &tileRange0(tileRange.append(Json::arrayValue));
    tileRange0.append(r.tileRange.ll(0));
    tileRange0.append(r.tileRange.ll(1));
    auto &tileRange1(tileRange.append(Json::arrayValue));
    tileRange1.append(r.tileRange.ur(0));
    tileRange1.append(r.tileRange.ur(1));

    value["definition"] = buildDefinition(r);
}

void saveResource(std::ostream &out, const Resource &resource)
{
    Json::Value value;
    buildResource(value, resource);

    out.precision(15);
    Json::write(out, value);
}

} // namespace detail

Resource::map loadResources(const boost::filesystem::path &path
                            , ResourceLoadErrorCallback
                            , const FileClassSettings &fileClassSettings)
{
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);

    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, IOError)
            << "Unable to load resources " << path
            << ": <" << e.what() << ">.";
    }

    return detail::loadResources(f, path, fileClassSettings);
}

Resource::list loadResource(const boost::filesystem::path &path
                            , const FileClassSettings &fileClassSettings)
{
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);

    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, IOError)
            << "Unable to load resource file " << path
            << ": <" << e.what() << ">.";
    }

    return detail::loadResource(f, path, fileClassSettings);
}

void save(const boost::filesystem::path &path, const Resource &resource)
{
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);

    try {
        f.open(path.string(), std::ios_base::out | std::ios_base::trunc);
    } catch (const std::exception &e) {
        LOGTHROW(err1, IOError)
            << "Unable to save resource file " << path
            << ": <" << e.what() << ">.";
    }

    detail::saveResource(f, resource);
}

Changed Resource::changed(const Resource &o) const
{
    // mandatory stuff first
    if (!(id == o.id)) { return Changed::yes; }
    if (!(generator == o.generator)) { return Changed::yes; }

    if (lodRange != o.lodRange) { return Changed::yes; }
    if (tileRange != o.tileRange) { return Changed::yes; }

    // compare credits only if frozen
    bool changedCredits(credits != o.credits);
    if (definition_->frozenCredits() && changedCredits) {
        return Changed::yes;
    }

    // TODO: registry ?

    // check definition, it must check mandatory stuff first, save stuff
    // second
    auto def(definition_->changed(*o.definition()));
    if (def != Changed::no) { return def; }

    // from here down only safely-changed stuff can follow

    if (changedCredits) { return Changed::safely; }

    // not changed at all
    return Changed::no;
}

boost::filesystem::path prependRoot(const boost::filesystem::path &path
                                    , const Resource::Id &resource
                                    , Resource::Generator::Type generatorType
                                    , const ResourceRoot &root)
{
    boost::filesystem::path out;

    // back-up given number of levels up the tree
    for (int i(root.backup); i > 0; --i) { out /= ".."; }

    switch (root) {
    case ResourceRoot::referenceFrame:
        out /= resource.referenceFrame;
        // fall through

    case ResourceRoot::type:
        out /= boost::lexical_cast<std::string>(generatorType);
        // fall through

    case ResourceRoot::group:
        out /= resource.group;
        // fall through

    case ResourceRoot::id:
        out /= resource.id;
        // fall through

    case ResourceRoot::none:
        // nothing
        break;
    }

    out /= path;
    return out;
}

std::string prependRoot(const std::string &path
                        , const Resource::Id &resource
                        , Resource::Generator::Type generatorType
                        , const ResourceRoot &root)
{
    fs::path tmp(path);
    return prependRoot(tmp, resource, generatorType, root).string();
}

ResourceRoot resolveRoot(const Resource::Id &thisResource
                         , Resource::Generator::Type thisGeneratorType
                         , const Resource::Id &thatResource
                         , Resource::Generator::Type thatGeneratorType
                         , ResourceRoot::Depth thisDepth)
{
    // compute difference between two resources
    auto difference([&]() -> ResourceRoot
    {
        if (thisResource.referenceFrame != thatResource.referenceFrame) {
            return { ResourceRoot::referenceFrame, 4 };
        }

        if (thisGeneratorType != thatGeneratorType) {
            return { ResourceRoot::type, 3 };
        }

        if (thisResource.group != thatResource.group) {
            return { ResourceRoot::group, 2 };
        }

        if (thisResource.id != thatResource.id) {
            return { ResourceRoot::id, 1 };
        }

        // nothing more
        return { ResourceRoot::none, 0 };
    }());

    if (thisDepth < difference.depth) {
        difference.backup -= (difference.depth - thisDepth);
    }
    return difference;
}

std::string contentType(RasterFormat format)
{
    switch (format) {
    case RasterFormat::jpg: return "image/jpeg";
    case RasterFormat::png: return "image/png";
    }
    return {};
}

bool checkRanges(const Resource &resource, const vts::TileId &tileId
                 , RangeType rangeType)
{
    if (!in(tileId.lod, resource.lodRange)) {
        return false;
    }

    // LOD is enough
    if (rangeType == RangeType::lod) { return true; }

    // tileId.lod is inside lorRange, so difference is always positive
    auto pTileId(vts::parent(tileId, tileId.lod - resource.lodRange.min));
    if (!inside(resource.tileRange, pTileId.x, pTileId.y)) {
        return false;
    }

    return true;
}

vr::Credits asInlineCredits(const Resource &res)
{
    vr::Credits credits;
    for (auto &id : res.credits) {
        const auto *credit(res.registry.credits(id.id, std::nothrow));
        if (!credit) { credit = vr::system.credits(id.id, std::nothrow); }
        if (credit) { credits.set(id, *credit); }
    }
    return credits;
}
