/**
 * Copyright (c) 2018 Melown Technologies SE
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

#include "utility/enum-io.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "heightfunction.hpp"

#include "error.hpp"

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

} // namespace

HeightFunction::pointer HeightFunction::parse(const Json::Value &value
                                              , const std::string &key)
{
    return parseJson(value, key);
}

void SuperElevation::build(Json::Value &value) const
{
    buildJson(value, config_);
}

bool HeightFunction::changed(const HeightFunction::pointer &l
                             , const HeightFunction::pointer &r)
{
    if (!l) { return bool(r); }
    return l->changed(r);
}
