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

#include "surface-meta.hpp"
#include "factory.hpp"

namespace resource {

constexpr char SurfaceMeta::driverName[];

MAPPROXY_DEFINITION_REGISTER(SurfaceMeta)

namespace {

void parseDefinition(SurfaceMeta &def, const Json::Value &value)
{
    const auto &rid([](const Json::Value &item) -> Resource::Id
    {
        Resource::Id id;
        Json::get(id.group, item, "group");
        Json::get(id.id, item, "id");
        return id;
    });

    def.surface = rid(Json::check(value["surface"], Json::objectValue));
    def.tms = rid(Json::check(value["tms"], Json::objectValue));
}

void buildDefinition(Json::Value &value, const SurfaceMeta &def)
{
    const auto &rid([](const Resource::Id &id) -> Json::Value
    {
        Json::Value value;
        value["group"] = id.group;
        value["id"] = id.id;
        return value;
    });

    value["surface"] = rid(def.surface);
    value["tms"] = rid(def.tms);
}

} // namespace

void SurfaceMeta::from_impl(const Json::Value &value)
{
    parseDefinition(*this, value);
}

void SurfaceMeta::to_impl(Json::Value &value) const
{
    buildDefinition(value, *this);
}

Changed SurfaceMeta::changed_impl(const DefinitionBase &o) const
{
    const auto &other(o.as<SurfaceMeta>());

    if (surface != other.surface) { return Changed::yes; }
    if (tms != other.tms) { return Changed::yes; }

    return Changed::no;
}

Resource::Id::list SurfaceMeta::needsResources_impl() const {
    return { surface, tms };
}

} // namespace resource
