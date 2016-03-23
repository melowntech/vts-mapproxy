#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/registry/json.hpp"

#include "./error.hpp"
#include "./resource.hpp"

namespace fs = boost::filesystem;

namespace vr = vadstena::registry;
namespace vs = vadstena::storage;

namespace resdef {

Resource::Generator TmsRaster::generator("tms", "tms-raster");
Resource::Generator SurfaceSpheroid::generator("surface", "surface-sphereoid");
Resource::Generator SurfaceDem::generator("surface", "surface-dem");

} // namespace resdef

namespace detail {

void parseCredits(vr::IdSet &ids, const Json::Value &object
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

        ids.insert(credit.numericId);
    }
}

void parseDefinition(resdef::TmsRaster &def, const Json::Value &value)
{
    std::string s;
    Json::get(s, value, "dataset"); def.datasetPath = s;
    Json::get(s, value, "mask"); def.maskPath = s;
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
    (void) def;
    (void) value;
}

void parseDefinition(resdef::SurfaceDem &def, const Json::Value &value)
{
    (void) def;
    (void) value;
}

template <typename T>
T& createDefinition(Resource &r)
{
    return boost::any_cast<T&>(r.definition = T());
}

void parseDefinition(Resource &r, const Json::Value &value)
{
    if (r.generator == resdef::TmsRaster::generator) {
        return parseDefinition
            (createDefinition<resdef::TmsRaster>(r), value);
    }
    if (r.generator == resdef::SurfaceSpheroid::generator) {
        return parseDefinition
            (createDefinition<resdef::SurfaceSpheroid>(r), value);
    }
    if (r.generator == resdef::SurfaceDem::generator) {
        return parseDefinition
            (createDefinition<resdef::SurfaceDem>(r), value);
    }

    LOGTHROW(err1, UnknownGenerator)
        << "Unknown generator <" << r.generator << ">.";
}

void buildDefinition(Json::Value &value, const resdef::TmsRaster &def)
{
    value["dataset"] = def.datasetPath.string();
    value["mask"] = def.maskPath.string();
    value["format"] = boost::lexical_cast<std::string>(def.format);
}

void buildDefinition(Json::Value &value, const resdef::SurfaceSpheroid &def)
{
    (void) value;
    (void) def;
}

void buildDefinition(Json::Value &value, const resdef::SurfaceDem &def)
{
    (void) value;
    (void) def;
}

template <typename T>
const T& getDefinition(const Resource &r)
{
    return boost::any_cast<const T&>(r.definition);
}

void buildDefinition(Json::Value &value, const Resource &r)
{
    if (r.generator == resdef::TmsRaster::generator) {
        return buildDefinition
            (value, getDefinition<resdef::TmsRaster>(r));
    }
    if (r.generator == resdef::SurfaceSpheroid::generator) {
        return buildDefinition
            (value, getDefinition<resdef::SurfaceSpheroid>(r));
    }
    if (r.generator == resdef::SurfaceDem::generator) {
        return buildDefinition
            (value, getDefinition<resdef::SurfaceDem>(r));
    }

    LOGTHROW(err1, UnknownGenerator)
        << "Unknown generator <" << r.generator << ">.";
}

Resource parseResource(const Json::Value &value)
{
    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Resource definition is not an object.";
    }

    Resource r;

    Json::get(r.id.group, value, "group");
    Json::get(r.id.id, value, "id");
    Json::get(r.generator.type, value, "type");
    Json::get(r.generator.driver, value, "driver");

    parseCredits(r.credits, value, "credits");

    const Json::Value &referenceFrames(value["referenceFrames"]);
    if (!referenceFrames.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Type of referenceFrames is not an object.";
    }

    for (const auto &name : referenceFrames.getMemberNames()) {
        const auto &content(referenceFrames[name]);

        auto &rfd(r.referenceFrames[name]);
        // NB: function either returns valid reference of throws
        rfd.referenceFrame = &vr::Registry::referenceFrame(name);

        Json::get(rfd.lodRange.min, content, "lodRange", 0);
        Json::get(rfd.lodRange.max, content, "lodRange", 1);
        rfd.tileRange = vr::tileRangeFromJson(content["tileRange"]);
    }

    parseDefinition(r, value["definition"]);

    return r;
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
        auto res(parseResource(item));

        if (!resources.insert(Resource::map::value_type(res.id, res)).second) {
            LOGTHROW(err1, Json::Error)
                << "Duplicate entry for <" << res.id << ">.";
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

Resource loadResource(std::istream &in, const fs::path &path)
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
    value["type"] = r.generator.type;
    value["driver"] = r.generator.driver;

    auto &credits(value["credits"] = Json::arrayValue);
    for (auto cid : r.credits) { credits.append(cid); }

    Json::Value &referenceFrames(value["referenceFrames"]);
    for (const auto &item : r.referenceFrames) {
        auto &content(referenceFrames[item.first] = Json::objectValue);
        const auto &rfd(item.second);

        auto &lodRange(content["lodRange"] = Json::arrayValue);
        lodRange.append(rfd.lodRange.min);
        lodRange.append(rfd.lodRange.max);

        auto &tileRange(content["tileRange"] = Json::arrayValue);
        auto &tileRange0(tileRange.append(Json::arrayValue));
        tileRange0.append(rfd.tileRange.ll(0));
        tileRange0.append(rfd.tileRange.ll(1));
        auto &tileRange1(tileRange.append(Json::arrayValue));
        tileRange1.append(rfd.tileRange.ur(0));
        tileRange1.append(rfd.tileRange.ur(1));
    }

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

Resource loadResource(const boost::filesystem::path &path)
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
    return (getDefinition<T>(l) == getDefinition<T>(r));
}

} // namespace detail

bool Resource::operator==(const Resource &o) const
{
    if (!(id == o.id)) { return false; }
    if (!(generator == o.generator)) { return false; }
    if (credits != o.credits) { return false; }
    if (referenceFrames != o.referenceFrames) { return false; }

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

bool Resource::ReferenceFrame::operator==(const ReferenceFrame &o) const
{
    if (lodRange != o.lodRange) { return false; }
    if (tileRange != o.tileRange) { return false; }
    return true;
}

namespace resdef {

bool TmsRaster::operator==(const TmsRaster &o) const
{
    // TODO: implement me
    (void) o;
    return true;
}

bool SurfaceSpheroid::operator==(const SurfaceSpheroid &o) const
{
    // TODO: implement me
    (void) o;
    return true;
}

bool SurfaceDem::operator==(const SurfaceDem &o) const
{
    // TODO: implement me
    (void) o;
    return true;
}

} // namespace resdef
