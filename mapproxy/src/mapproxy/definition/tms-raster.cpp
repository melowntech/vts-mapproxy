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
#include <boost/utility/in_place_factory.hpp>

#include "utility/premain.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "tms.hpp"
#include "factory.hpp"

namespace resource {

constexpr char TmsRaster::driverName[];

MAPPROXY_DEFINITION_REGISTER(TmsRaster)

namespace {

void parseDefinition(TmsRaster &def, const Json::Value &value)
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
        } catch (const boost::bad_lexical_cast&) {
            utility::raise<Json::Error>
                ("Value stored in format is not RasterFormat value");
        }
    }

    if (value.isMember("transparent")) {
        Json::get(def.transparent, value, "transparent");
    }

    Json::get(def.resampling, value, "resampling");

    def.parse(value);
}

void buildDefinition(Json::Value &value, const TmsRaster &def)
{
    value["dataset"] = def.dataset;
    if (def.mask) {
        value["mask"] = *def.mask;
    }
    value["format"] = boost::lexical_cast<std::string>(def.format);

    value["transparent"] = def.transparent;

    if (def.resampling) {
        value["resampling"]
            = boost::lexical_cast<std::string>(*def.resampling);
    }

    def.build(value);
}

} // namespace

void TmsRaster::from_impl(const Json::Value &value)
{
    parseDefinition(*this, value);
}

void TmsRaster::to_impl(Json::Value &value) const
{
    buildDefinition(value, *this);
}

Changed TmsRaster::changed_impl(const DefinitionBase &o) const
{
    const auto &other(o.as<TmsRaster>());

    // non-safe changes first
    if (dataset != other.dataset) { return Changed::yes; }
    if (mask != other.mask) { return Changed::yes; }

    // transparent can change
    if (transparent != other.transparent) { return Changed::safely; }

    // format can change
    if (format != other.format) { return Changed::safely; }

    // format can change
    if (resampling != other.resampling) { return Changed::safely; }

    return TmsCommon::changed_impl(o);
}

} // namespace resource
