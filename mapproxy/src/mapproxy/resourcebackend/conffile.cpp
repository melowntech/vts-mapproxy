#include "dbglog/dbglog.hpp"

#include "utility/premain.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/registry/json.hpp"

#include "../error.hpp"
#include "./conffile.hpp"
#include "./factory.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace vr = vadstena::registry;
namespace vs = vadstena::storage;

namespace resource_backend {

namespace {

struct Factory : ResourceBackend::Factory {
    virtual ResourceBackend::pointer create(const TypedConfig &config)
    {
        return std::make_shared<Conffile>(config.value<Conffile::Config>());
    }

    virtual service::UnrecognizedParser::optional
    configure(const std::string &prefix, TypedConfig &typedConfig)
    {
        auto &config(typedConfig.assign<Conffile::Config>());

        service::UnrecognizedParser parser
            ("resource backend " + typedConfig.type + ": "
             "configuration file-based resource backend");
        parser.options.add_options()
            ((prefix + "path").c_str()
             , po::value(&config.path)->required()
             , "Path to resource file (JSON).");

        return parser;
    }

    void printConfig(std::ostream &os, const std::string &prefix
                     , const TypedConfig &typedConfig)
    {
        const auto &config(typedConfig.value<Conffile::Config>());

        os << prefix << "path = " << config.path << "\n";
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    ResourceBackend::registerType("conffile", std::make_shared<Factory>());
});

} // namespace

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

Resource parseResource(const Json::Value &value)
{
    if (!value.isObject()) {
        LOGTHROW(err1, Json::Error)
            << "Resource definition is not an object";
    }

    Resource r;

    Json::get(r.group, value, "group");
    Json::get(r.id, value, "id");
    Json::get(r.id, value, "type");

    parseCredits(r.credits, value, "credits");

    // path template is optional
    if (value.isMember("path")) {
        Json::get(r.pathTemplate, value, "path");
    }

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

        // TODO: parse definition
    }

    return r;
}

void parseGroups(Resource::Groups &groups, const Json::Value &value)
{
    if (!value.isArray()) {
        LOGTHROW(err1, Json::Error)
            << "Type of top-level configuration is not a list.";
    }

    // process all definitions
    for (const auto &item : value) {
        // parse resource and remember
        auto res(parseResource(item));

        auto &group(groups[res.group]);
        if (!group.insert(Resource::Group::value_type(res.id, res)).second) {
            LOGTHROW(err1, Json::Error)
                << "Duplicate entry for <" << res.group << ">/<"
                << res.id << ">.";
        }
    }
}

Resource::Groups loadConfig(std::istream &in, const fs::path &path)
{
    Json::Value config;
    Json::Reader reader;
    if (!reader.parse(in, config)) {
        LOGTHROW(err2, FormatError)
            << "Unable to parse resource config " << path << ": <"
            << reader.getFormattedErrorMessages() << ">.";
    }

    Resource::Groups groups;
    try {
        parseGroups(groups, config);
    } catch (const Json::Error &e) {
        LOGTHROW(err1, FormatError)
            << "Invalid resource config file " << path
            << " format: <" << e.what() << ">.";
    } catch (const vs::Error &e) {
        LOGTHROW(err1, FormatError)
            << "Invalid resource config file " << path
            << " format: <" << e.what() << ">.";
    }

    return groups;
}

Resource::Groups loadConfig(const fs::path &path)
{
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);

    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, IOError)
            << "Unable to load resource config file " << path
            << ": <" << e.what() << ">.";
    }

    return loadConfig(f, path);
}

} // namespace detail

Conffile::Conffile(const Config &config)
    : config_(config)
{
    // try to load config file now
    load_impl();
}

Resource::Groups Conffile::load_impl() const
{
    return detail::loadConfig(config_.path);
}

} // namespace resource_backend
