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

#include "tms.hpp"
#include "factory.hpp"

namespace resource {

constexpr char TmsRasterSolid::driverName[];

MAPPROXY_DEFINITION_REGISTER(TmsRasterSolid)

namespace {

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

} // namespace

void TmsRasterSolid::from_impl(const Json::Value &value)
{
    parseDefinition(*this, value);
}

void TmsRasterSolid::to_impl(Json::Value &value) const
{
    buildDefinition(value, *this);
}

Changed TmsRasterSolid::changed_impl(const DefinitionBase &o) const
{
    const auto &other(o.as<TmsRasterSolid>());

    if (color != other.color) { return Changed::yes; }

    return TmsRasterSynthetic::changed_impl(other);
}

} // namespace resource
