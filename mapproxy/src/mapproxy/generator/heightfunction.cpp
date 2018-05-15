#include <boost/lexical_cast.hpp>

#include "utility/enum-io.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "./heightfunction.hpp"

#include "../error.hpp"
#include "../support/python.hpp"

SuperElevation::SuperElevation(const Config &config)
    : config_(config), s_()
{
    if (config_.heightRange.min >= config_.heightRange.max) {
        LOGTHROW(err2, std::logic_error)
            << "Invalid superelevation configuration (height range is "
            << config_.heightRange << ").";
    }

    s_ = ((config_.scaleRange.max - config_.scaleRange.min)
          / (config_.heightRange.max - config_.heightRange.min));
    limits_.min = apply(config_.heightRange.min);
    limits_.max = apply(config_.heightRange.max);
}

bool SuperElevation::Config::changed(const Config &other) const
{
    if (heightRange != other.heightRange) { return true; }
    if (scaleRange != other.scaleRange) { return true; }
    return false;
}

bool SuperElevation::changed(const HeightFunction::pointer &other) const
{
    if (!other) { return true; }
    if (const auto *o = dynamic_cast<const SuperElevation*>(other.get())) {
        // same type compare configs
        return config_.changed(o->config_);
    }
    // different type
    return true;
}

namespace {

UTILITY_GENERATE_ENUM(HeightFunctionType,
    ((superelevation))
)

void parse(SuperElevation::Config &config, const Json::Value &value)
{
    Json::get(config.heightRange.min, value, "heightRange", 0);
    Json::get(config.heightRange.max, value, "heightRange", 1);
    Json::get(config.scaleRange.min, value, "scaleRange", 0);
    Json::get(config.scaleRange.max, value, "scaleRange", 1);
}

void buildJson(Json::Value &value, const SuperElevation::Config &config)
{
    value["function"] = boost::lexical_cast<std::string>
        (HeightFunctionType::superelevation);

    auto &heightRange(value["heightRange"] = Json::arrayValue);
    heightRange.append(config.heightRange.min);
    heightRange.append(config.heightRange.max);

    auto &scaleRange(value["scaleRange"] = Json::arrayValue);
    scaleRange.append(config.scaleRange.min);
    scaleRange.append(config.scaleRange.max);
}

void parse(SuperElevation::Config &config, const boost::python::dict &value)
{
    const auto &heightRange(value["heightRange"]);
    config.heightRange.min
        = boost::python::extract<double>(heightRange[0]);
    config.heightRange.max
        = boost::python::extract<double>(heightRange[1]);

    const auto &scaleRange(value["scaleRange"]);
    config.scaleRange.min
        = boost::python::extract<double>(scaleRange[0]);
    config.scaleRange.max
        = boost::python::extract<double>(scaleRange[1]);
}

HeightFunction::pointer
parseJson(const Json::Value &value, const std::string &key)
{
    if (!value.isMember(key)) { return {}; }

    const auto &content(value[key]);

    HeightFunctionType type{};
    Json::get(type, content, "function");

    switch (type) {
    case HeightFunctionType::superelevation: {
        SuperElevation::Config config;
        parse(config, content);
        return std::make_shared<SuperElevation>(config);
    } break;
    }

    // never reached
    throw;
}

HeightFunction::pointer parsePy(const boost::python::dict &value
                                , const std::string &key)
{
    if (!value.has_key(key)) { return {}; }

    boost::python::dict content(value[key]);

    const auto type(boost::lexical_cast<HeightFunctionType>
                    (py2utf8(content["function"])));

    switch (type) {
    case HeightFunctionType::superelevation: {
        SuperElevation::Config config;
        parse(config, content);
        return std::make_shared<SuperElevation>(config);
    } break;
    }

    // never reached
    throw;
}

} // namespace

HeightFunction::pointer HeightFunction::parse(const boost::any &value
                                              , const std::string &key)
{
    if (const auto *json = boost::any_cast<Json::Value>(&value)) {
        return parseJson(*json, key);
    } else if (const auto *py = boost::any_cast<boost::python::dict>(&value)) {
        return parsePy(*py, key);
    }

    LOGTHROW(err1, Error)
        << "Function: Unsupported configuration from: <"
        << value.type().name() << ">.";
    throw;
}

void SuperElevation::build(boost::any &value) const
{
    if (auto *json = boost::any_cast<Json::Value>(&value)) {
        buildJson(*json, config_);
    } else {
        LOGTHROW(err1, Error)
            << "SuperElevation:: Unsupported serialization into: <"
            << value.type().name() << ">.";
    }
}

bool HeightFunction::changed(const HeightFunction::pointer &l
                             , const HeightFunction::pointer &r)
{
    if (!l) { return bool(r); }
    return l->changed(r);
}
