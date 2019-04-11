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

#include "utility/premain.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "../support/python.hpp"

#include "tms.hpp"
#include "factory.hpp"

namespace resource {

constexpr char TmsRasterSolid::driverName[];

namespace {

utility::PreMain register_([]() { registerDefinition<TmsRasterSolid>(); });

void parseDefinition(TmsRasterSolid &def, const Json::Value &value)
{
    const auto &color(Json::check(value["color"], Json::arrayValue));

    Json::get(def.color[0], color, 0, "definition.color");
    Json::get(def.color[1], color, 1, "definition.color");
    Json::get(def.color[2], color, 2, "definition.color");

    def.parse(value);
}

void buildDefinition(Json::Value &value, const TmsRasterSolid &def)
{
    auto &color(value["color"] = Json::arrayValue);

    color.append(def.color[0]);
    color.append(def.color[1]);
    color.append(def.color[2]);

    def.build(value);
}

void parseDefinition(TmsRasterSolid &def
                     , const boost::python::dict &value)
{
    namespace python = boost::python;

    boost::python::list color(value["color"]);

    def.color[0] = boost::python::extract<int>(color[0]);
    def.color[1] = boost::python::extract<int>(color[1]);
    def.color[2] = boost::python::extract<int>(color[2]);

    def.parse(value);
}

} // namespace

void TmsRasterSolid::from_impl(const boost::any &value)
{
    if (const auto *json = boost::any_cast<Json::Value>(&value)) {
        parseDefinition(*this, *json);
    } else if (const auto *py
               = boost::any_cast<boost::python::dict>(&value))
    {
        parseDefinition(*this, *py);
    } else {
        LOGTHROW(err1, Error)
            << "TmsRasterSolid: Unsupported configuration from: <"
            << value.type().name() << ">.";
    }
}

void TmsRasterSolid::to_impl(boost::any &value) const
{
    if (auto *json = boost::any_cast<Json::Value>(&value)) {
        buildDefinition(*json, *this);
    } else {
        LOGTHROW(err1, Error)
            << "TmsRasterSolid: Unsupported serialization into: <"
            << value.type().name() << ">.";
    }
}

Changed TmsRasterSolid::changed_impl(const DefinitionBase &o) const
{
    const auto &other(o.as<TmsRasterSolid>());

    if (color != other.color) { return Changed::yes; }

    return TmsRasterSynthetic::changed_impl(other);
}

} // namespace resource
