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

#include "geodata-semantic.hpp"
#include "factory.hpp"
#include "options.hpp"
#include "parse.hpp"

namespace vf = geo::vectorformat;

namespace resource {

constexpr Resource::Generator::Type GeodataSemantic::type;
constexpr char GeodataSemantic::driverName[];

namespace {

utility::PreMain register_([]() { registerDefinition<GeodataSemantic>(); });

void parseDefinition(GeodataSemanticBase &def, const Json::Value &value)
{

    Json::get(def.dataset, value, "dataset");

    Json::getOpt(def.format, value, "format");
    if (value.isMember("formatConfig")) {
        parse(def.formatConfig, def.format, value["formatConfig"]);
    }

    Json::getOpt(def.styleUrl, value, "styleUrl");
    Json::get(def.displaySize, value, "displaySize");

    if (value.isMember("options")) { def.options = value["options"]; }

    if (value.isMember("introspection")) {
        def.introspection.parse(value["introspection"]);
    }
}

void buildDefinition(Json::Value &value, const GeodataSemanticBase &def)
{
    value["dataset"] = def.dataset;

    value["format"] = boost::lexical_cast<std::string>(def.format);
    build(value["formatConfig"], def.formatConfig);

    value["displaySize"] = def.displaySize;
    value["styleUrl"] = def.styleUrl;

    if (!def.options.empty()) {
        value["options"] = boost::any_cast<Json::Value>(def.options);
    }

    if (!def.introspection.empty()) {
        def.introspection.build(value["introspection"]);
    }
}

} // namespace

void GeodataSemanticBase::from_impl(const Json::Value &value)
{
    parseDefinition(*this, value);
}

void GeodataSemanticBase::to_impl(Json::Value &value) const
{
    buildDefinition(value, *this);
}

Changed GeodataSemanticBase::changed_impl(const DefinitionBase &o) const
{
    const auto &other(o.as<GeodataSemanticBase>());

    // accumulate revision bump/safe changes
    bool bump(false);
    bool safe(false);

    // changing these bumps version
    if (dataset != other.dataset) { bump = true; }

    // different format leads to version bump!
    if (format != other.format) { bump = true; }
    // different format config leads to version bump!
    if (differ(formatConfig, other.formatConfig)) { bump = true; }
    // displaySize can change
    if (displaySize != other.displaySize) { safe = true; }

    // styleUrl can change
    if (styleUrl != other.styleUrl) { safe = true; }

    // options can be safely changed
    if (optionsChanged(*this, other)) { safe = true; }

    // introspection can change
    if (introspection != other.introspection) { safe = true; }

    // interpret change type
    if (bump) { return Changed::withRevisionBump; }
    if (safe) { return Changed::safely; }
    return Changed::no;
}

} // namespace resource
