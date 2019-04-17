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

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/registry/json.hpp"
#include "vts-libs/registry/py.hpp"

#include "../support/introspection.hpp"

#include "geodata.hpp"
#include "options.hpp"
#include "parse.hpp"

namespace vf = geo::vectorformat;

namespace resource {

namespace {

void parseLayers(boost::optional<geo::heightcoding::Config::LayerNames> &result
                 , const std::string &name, const Json::Value &value)
{
    if (!value.isMember(name)) { return; }

    const auto layers(value[name]);
    if (!layers.isArray()) {
        LOGTHROW(err1, FormatError)
            << "Geodata definition[layers] is not an array.";
    }

    result = boost::in_place();
    for (const auto &layer : layers) {
        result->push_back(layer.asString());
    }
    std::sort(result->begin(), result->end());
}

void parseDefinition(GeodataVectorBase &def
                     , const Json::Value &value)
{
    std::string s;

    Json::get(def.dataset, value, "dataset");
    Json::get(def.dem.dataset, value, "demDataset");

    if (value.isMember("geoidGrid")) {
        def.dem.geoidGrid = boost::in_place();
        Json::get(*def.dem.geoidGrid, value, "geoidGrid");
    }

    parseLayers(def.layers, "layers", value);
    parseLayers(def.clipLayers, "clipLayers", value);

    Json::getOpt(def.format, value, "format");

    if (value.isMember("formatConfig")) {
        parse(def.formatConfig, def.format, value["formatConfig"]);
    }

    Json::getOpt(def.styleUrl, value, "styleUrl");
    Json::get(def.displaySize, value, "displaySize");
    Json::getOpt(def.mode, value, "mode");

    if (value.isMember("enhance")) {
        const auto enhance(value["enhance"]);
        if (!enhance.isObject()) {
            LOGTHROW(err1, FormatError)
                << "Geodata definition[enhance] is not an object.";
        }

        for (const auto &layerName : enhance.getMemberNames()) {
            const auto &layer(enhance[layerName]);
            auto &lh(def.layerEnhancers[layerName]);
            Json::get(lh.key, layer, "key");
            Json::get(lh.databasePath, layer, "db");
            Json::get(lh.table, layer, "table");
        }
    }

    def.heightFunction = HeightFunction::parse(value, "heightFunction");

    if (value.isMember("options")) { def.options = value["options"]; }

    if (value.isMember("introspection")) {
        def.introspection.parse(value["introspection"]);
    }
}

void buildLayers(const boost::optional
                 <geo::heightcoding::Config::LayerNames> &layers
                 , const std::string &name, Json::Value &value)
{
    if (!layers) { return; }
    auto &players(value[name] = Json::arrayValue);
    for (const auto &layer : *layers) {
        players.append(layer);
    }
}

void buildDefinition(Json::Value &value
                     , const GeodataVectorBase &def)
{
    value["dataset"] = def.dataset;
    value["demDataset"] = def.dem.dataset;

    if (def.dem.geoidGrid) {
        value["geoidGrid"] = *def.dem.geoidGrid;
    }

    buildLayers(def.layers, "layers", value);
    buildLayers(def.clipLayers, "clipLayers", value);

    value["format"] = boost::lexical_cast<std::string>(def.format);
    build(value["formatConfig"], def.formatConfig);

    value["displaySize"] = def.displaySize;
    value["styleUrl"] = def.styleUrl;
    value["mode"] = boost::lexical_cast<std::string>(def.mode);

    if (!def.layerEnhancers.empty()) {
        auto &layerEnhancers(value["enhance"] = Json::objectValue);
        for (const auto &item : def.layerEnhancers) {
            auto &layer(layerEnhancers[item.first] = Json::objectValue);
            layer["key"] = item.second.key;
            layer["db"] = item.second.databasePath;
            layer["table"] = item.second.table;
        }
    }

    if (def.heightFunction) {
        auto &tmp(value["heightFunction"] = Json::objectValue);
        def.heightFunction->build(tmp);
    }

    if (!def.options.empty()) {
        value["options"] = boost::any_cast<Json::Value>(def.options);
    }

    if (!def.introspection.empty()) {
        def.introspection.build(value["introspection"]);
    }
}

} // namespace

void GeodataVectorBase::from_impl(const Json::Value &value)
{
    parseDefinition(*this, value);
}

void GeodataVectorBase::to_impl(Json::Value &value) const
{
    buildDefinition(value, *this);
}

Changed GeodataVectorBase::changed_impl(const DefinitionBase &o) const
{
    const auto &other(o.as<GeodataVectorBase>());

    // accumulate revision bump/safe changes
    bool bump(false);
    bool safe(false);

    // changing these bumps version
    if (dem != other.dem) { bump = true; }
    if (dataset != other.dataset) { bump = true; }
    if (layers != other.layers) { bump = true; }
    if (clipLayers != other.clipLayers) { bump = true; }
    if (mode != other.mode) { bump = true; }
    if (layerEnhancers != other.layerEnhancers) { bump = true; }
    if (HeightFunction::changed(heightFunction, other.heightFunction)) {
        bump = true;
    }

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
