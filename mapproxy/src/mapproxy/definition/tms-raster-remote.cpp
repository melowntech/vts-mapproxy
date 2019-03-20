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

#include "../support/python.hpp"

#include "./tms.hpp"
#include "./factory.hpp"

namespace resource {

constexpr char TmsRasterRemote::driverName[];

namespace {

utility::PreMain register_([]() { registerDefinition<TmsRasterRemote>(); });

void parseDefinition(TmsRasterRemote &def
                     , const Json::Value &value)
{
    std::string s;

    Json::get(def.remoteUrl, value, "remoteUrl");
    if (value.isMember("mask")) {
        std::string s;
        Json::get(s, value, "mask");
        def.mask = s;;
    }

    def.parse(value);
}

void buildDefinition(Json::Value &value
                     , const TmsRasterRemote &def)
{
    value["remoteUrl"] = def.remoteUrl;
    if (def.mask) {
        value["mask"] = def.mask->string();
    }

    def.build(value);
}

void parseDefinition(TmsRasterRemote &def
                     , const boost::python::dict &value)
{
    def.remoteUrl = py2utf8(value["remoteUrl"]);

    if (value.has_key("mask")) {
        def.mask = py2utf8(value["mask"]);
    }

    def.parse(value);
}

} // namespace

void TmsRasterRemote::from_impl(const boost::any &value)
{
    if (const auto *json = boost::any_cast<Json::Value>(&value)) {
        parseDefinition(*this, *json);
    } else if (const auto *py
               = boost::any_cast<boost::python::dict>(&value))
    {
        parseDefinition(*this, *py);
    } else {
        LOGTHROW(err1, Error)
            << "TmsRasterRemote: Unsupported configuration from: <"
            << value.type().name() << ">.";
    }
}

void TmsRasterRemote::to_impl(boost::any &value) const
{
    if (auto *json = boost::any_cast<Json::Value>(&value)) {
        buildDefinition(*json, *this);
    } else {
        LOGTHROW(err1, Error)
            << "TmsRasterRemote:: Unsupported serialization into: <"
            << value.type().name() << ">.";
    }
}

Changed TmsRasterRemote::changed_impl(const DefinitionBase &o)
    const
{
    const auto &other(o.as<TmsRasterRemote>());

    if (remoteUrl != other.remoteUrl) { return Changed::yes; }
    if (mask != other.mask) { return Changed::yes; }

    return TmsCommon::changed_impl(o);
}

} // namespace resource
