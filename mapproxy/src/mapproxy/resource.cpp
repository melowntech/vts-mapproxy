#include <boost/lexical_cast.hpp>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/registry/json.hpp"
#include "vts-libs/vts/tileop.hpp"

#include "./error.hpp"
#include "./resource.hpp"

namespace fs = boost::filesystem;

namespace vr = vadstena::registry;
namespace vs = vadstena::storage;

namespace resdef {

Resource::Generator TmsRaster::generator
    (Resource::Generator::Type::tms, "tms-raster");
Resource::Generator SurfaceSpheroid::generator
    (Resource::Generator::Type::surface, "surface-spheroid");
Resource::Generator SurfaceDem::generator
    (Resource::Generator::Type::surface, "surface-dem");

} // namespace resdef

namespace detail {

void parseCredits(DualId::set &ids, const Json::Value &object
                  , const char *name)
{
    const Json::Value &value(object[name]);

    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of " << name << " is not an array.";
    }

    for (const auto &element : value) {
        const auto &credit([&]() -> const vr::Credit&
        {
            if (element.isIntegral()) {
                return vr::Registry::credit(element.asInt());
            }

            Json::check(element, Json::stringValue);
            return vr::Registry::credit(element.asString());
        }());

        ids.insert(DualId(credit.id, credit.numericId));
    }
}

void parseDefinition(resdef::TmsRaster &def, const Json::Value &value)
{
    std::string s;

    Json::get(def.dataset, value, "dataset");
    if (value.isMember("mask")) {
        def.mask = boost::in_place();
        Json::get(*def.mask, value, "mask");
    }

    if (value.isMember("format")) {
        Json::get(s, value, "format");
        try {
            def.format = boost::lexical_cast<RasterFormat>(s);
        } catch (boost::bad_lexical_cast) {
            utility::raise<Json::Error>
                ("Value stored in format is not RasterFormat value");
        }
    }
}

void parseDefinition(resdef::SurfaceSpheroid &def, const Json::Value &value)
{
    if (value.isMember("textureLayerId")) {
        Json::get(def.textureLayerId, value, "textureLayerId");
    }
    if (value.isMember("geoidGrid")) {
        std::string s;
        Json::get(s, value, "geoidGrid");
        def.geoidGrid = s;
    }
}

void parseDefinition(resdef::SurfaceDem &def, const Json::Value &value)
{
    Json::get(def.dataset, value, "dataset");
    if (value.isMember("mask")) {
        def.mask = boost::in_place();
        Json::get(*def.mask, value, "mask");
    }

    if (value.isMember("textureLayerId")) {
        Json::get(def.textureLayerId, value, "textureLayerId");
    }

    if (value.isMember("geoidGrid")) {
        std::string s;
        Json::get(s, value, "geoidGrid");
        def.geoidGrid = s;
    }
}

void parseDefinition(Resource &r, const Json::Value &value)
{
    if (r.generator == resdef::TmsRaster::generator) {
        return parseDefinition
            (r.assignDefinition<resdef::TmsRaster>(), value);
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

void buildDefinition(Json::Value &value, const resdef::TmsRaster &def)
{
    value["dataset"] = def.dataset;
    if (def.mask) {
        value["mask"] = *def.mask;
    }
    value["format"] = boost::lexical_cast<std::string>(def.format);
}

void buildDefinition(Json::Value &value, const resdef::SurfaceSpheroid &def)
{
    if (def.textureLayerId) {
        value["textureLayerId"] = def.textureLayerId;
    }
    if (def.geoidGrid) {
        value["geoidGrid"] = *def.geoidGrid;
    }
}

void buildDefinition(Json::Value &value, const resdef::SurfaceDem &def)
{
    value["dataset"] = def.dataset;
    if (def.mask) {
        value["mask"] = *def.mask;
    }

    if (def.textureLayerId) {
        value["textureLayerId"] = def.textureLayerId;
    }
    if (def.geoidGrid) {
        value["geoidGrid"] = *def.geoidGrid;
    }
}

void buildDefinition(Json::Value &value, const Resource &r)
{
    if (r.generator == resdef::TmsRaster::generator) {
        return buildDefinition
            (value, r.definition<resdef::TmsRaster>());
    }
    if (r.generator == resdef::SurfaceSpheroid::generator) {
        return buildDefinition
            (value, r.definition<resdef::SurfaceSpheroid>());
    }
    if (r.generator == resdef::SurfaceDem::generator) {
        return buildDefinition
            (value, r.definition<resdef::SurfaceDem>());
    }

    LOGTHROW(err1, UnknownGenerator)
        << "Unknown generator <" << r.generator << ">.";
}

Resource::list parseResource(const Json::Value &value)
{
    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Resource definition is not an object.";
    }

    Resource r;

    Json::get(r.id.group, value, "group");
    Json::get(r.id.id, value, "id");
    std::string tmp;
    Json::get(tmp, value, "type");
    r.generator.type = boost::lexical_cast<Resource::Generator::Type>(tmp);
    Json::get(r.generator.driver, value, "driver");

    parseCredits(r.credits, value, "credits");

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
        rr.referenceFrame = &vr::Registry::referenceFrame(name);

        Json::get(rr.lodRange.min, content, "lodRange", 0);
        Json::get(rr.lodRange.max, content, "lodRange", 1);
        rr.tileRange = vr::tileRangeFromJson(content["tileRange"]);
    }

    return out;
}

void parseResources(Resource::map &resources, const Json::Value &value)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of top-level configuration is not a list.";
    }

    // process all definitions
    for (const auto &item : value) {
        // parse resource and remember
        auto resList(parseResource(item));

        for (const auto &res : resList) {
            if (!resources.insert(Resource::map::value_type(res.id, res))
                .second)
            {
                LOGTHROW(err1, Json::Error)
                    << "Duplicate entry for <" << res.id << ">.";
            }
        }
    }
}

Resource::map loadResources(std::istream &in, const fs::path &path)
{
    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(in, config)) {
        LOGTHROW(err2, FormatError)
            << "Unable to parse resource file " << path << ": <"
            << reader.getFormattedErrorMessages() << ">.";
    }

    Resource::map resources;

    try {
        parseResources(resources, config);
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

Resource::list loadResource(std::istream &in, const fs::path &path)
{
    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(in, config)) {
        LOGTHROW(err2, FormatError)
            << "Unable to parse resource file " << path << ": <"
            << reader.getFormattedErrorMessages() << ">.";
    }

    try {
        return parseResource(config);
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

    buildDefinition(value["definition"], r);
}

void saveResource(std::ostream &out, const Resource &resource)
{
    Json::Value value;
    buildResource(value, resource);

    out.precision(15);
    Json::StyledStreamWriter().write(out, value);
}

} // namespace detail

Resource::map loadResources(const boost::filesystem::path &path)
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

    return detail::loadResources(f, path);
}

Resource::list loadResource(const boost::filesystem::path &path)
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

    return detail::loadResource(f, path);
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

namespace detail {

template<typename T>
bool sameDefinition(const Resource &l, const Resource &r)
{
    return (l.definition<T>() == r.definition<T>());
}

} // namespace detail

bool Resource::operator==(const Resource &o) const
{
    if (!(id == o.id)) { return false; }
    if (!(generator == o.generator)) { return false; }
    if (credits != o.credits) { return false; }

    if (lodRange != o.lodRange) { return false; }
    if (tileRange != o.tileRange) { return false; }

    if (generator == resdef::TmsRaster::generator) {
        if (!detail::sameDefinition<resdef::TmsRaster>(*this, o)) {
            return false;
        }
    } else if (generator == resdef::SurfaceSpheroid::generator) {
        if (!detail::sameDefinition<resdef::SurfaceSpheroid>(*this, o)) {
            return false;
        }
    } else if (generator == resdef::SurfaceDem::generator) {
        if (!detail::sameDefinition<resdef::SurfaceDem>(*this, o)) {
            return false;
        }
    }

    return true;
}

namespace resdef {

bool TmsRaster::operator==(const TmsRaster &o) const
{
    if (dataset != o.dataset) { return false; }
    if (mask != o.mask) { return false; }

    // format can change
    return true;
}

bool SurfaceSpheroid::operator==(const SurfaceSpheroid &o) const
{
    if (textureLayerId != o.textureLayerId) { return false; }
    if (geoidGrid != o.geoidGrid) { return false; }
    return true;
}

bool SurfaceDem::operator==(const SurfaceDem &o) const
{
    if (dataset != o.dataset) { return false; }
    if (mask != o.mask) { return false; }
    if (textureLayerId != o.textureLayerId) { return false; }
    if (geoidGrid != o.geoidGrid) { return false; }

    return true;
}

} // namespace resdef

boost::filesystem::path prependRoot(const boost::filesystem::path &path
                                    , const Resource &resource
                                    , ResourceRoot root)
{
    boost::filesystem::path out;

    switch (root) {
    case ResourceRoot::referenceFrame:
        out /= resource.id.referenceFrame;
        // no fallback

    case ResourceRoot::type:
        out /= boost::lexical_cast<std::string>(resource.generator.type);
        // no fallback

    case ResourceRoot::group:
        out /= resource.id.group;
        // no fallback

    case ResourceRoot::id:
        out /= resource.id.id;
        // no fallback

    case ResourceRoot::none:
        // nothing
        break;
    }

    out /= path;
    return out;
}

std::string prependRoot(const std::string &path
                        , const Resource &resource
                        , ResourceRoot root)
{
    fs::path tmp(path);
    return prependRoot(tmp, resource, root).string();
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
