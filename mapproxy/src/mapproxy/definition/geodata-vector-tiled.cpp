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

#include "vts-libs/registry/json.hpp"
#include "vts-libs/registry/py.hpp"

#include "geodata.hpp"
#include "factory.hpp"

namespace resource {

constexpr Resource::Generator::Type GeodataVectorTiled::type;
constexpr char GeodataVectorTiled::driverName[];

namespace {

utility::PreMain register_([]() { registerDefinition<GeodataVectorTiled>(); });

void parseDefinition(GeodataVectorTiled &def, const Json::Value &value)
{
    if (value.isMember("maxSourceLod")) {
        def.maxSourceLod = boost::in_place();
        Json::get(*def.maxSourceLod, value, "maxSourceLod");
    }
}

void buildDefinition(Json::Value &value, const GeodataVectorTiled &def)
{
    if (def.maxSourceLod) {
        value["maxSourceLod"] = *def.maxSourceLod;
    }
}

} // namespace

void GeodataVectorTiled::from_impl(const Json::Value &value)
{
    // deserialize parent class first
    GeodataVectorBase::from_impl(value);

    parseDefinition(*this, value);
}

void GeodataVectorTiled::to_impl(Json::Value &value) const
{
    // serialize parent class first
    GeodataVectorBase::to_impl(value);

    buildDefinition(value, *this);
}

Changed GeodataVectorTiled::changed_impl(const DefinitionBase &o)
    const
{
    // first check parent class for change
    const auto changed(GeodataVectorBase::changed_impl(o));
    if (changed == Changed::yes) { return changed; }

    const auto &other(o.as<GeodataVectorTiled>());

    // max source lod leads to revision bump
    if (maxSourceLod != other.maxSourceLod) {
        return Changed::withRevisionBump;
    }

    // pass result from parent
    return changed;
}

} // namespace resource
