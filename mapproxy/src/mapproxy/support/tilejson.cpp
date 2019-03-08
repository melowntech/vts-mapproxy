/**
 * Copyright (c) 2019 Melown Technologies SE
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

#include "utility/format.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/io.hpp"

#include "tilejson.hpp"

namespace {

void build(Json::Value &content, const LayerJson::Version &version)
{
    content = utility::format
        ("%d.%d.%d", version.maj, version.min, version.patch);
}

void build(Json::Value &content, const char *key
           , const LayerJson::OString &value)
{
    if (value) { content[key] = *value; }
}

void build(Json::Value &content, const math::Extents2 &extents)
{
    content = Json::arrayValue;
    content.append(extents.ll(0));
    content.append(extents.ll(1));
    content.append(extents.ur(0));
    content.append(extents.ur(1));
}

void build(Json::Value &content, const std::vector<std::string> &list)
{
    content = Json::arrayValue;
    for (const auto &string : list) { content.append(string); }
}

void build(Json::Value &content, const vts::TileRange &tileRange)
{
    content = Json::objectValue;
    content["startX"] = tileRange.ll(0);
    content["startY"] = tileRange.ll(1);
    content["endX"] = tileRange.ur(0);
    content["endY"] = tileRange.ur(1);
}

void build(Json::Value &content, const LayerJson::TileRanges &tileRanges)
{
    content = Json::arrayValue;
    for (const auto &tileRange : tileRanges) {
        build(content.append(Json::objectValue), tileRange);
    }
}

void build(Json::Value &content, const LayerJson::Available &available)
{
    content = Json::arrayValue;
    for (const auto &tileRanges : available) {
        build(content.append(Json::arrayValue), tileRanges);
    }
}

void build(Json::Value &content, const LayerJson &layer)
{
    content = Json::objectValue;
    build(content["tilejson"], layer.tilejson);
    build(content["version"], layer.version);
    build(content, "format", layer.format);
    build(content, "name", layer.name);
    build(content, "description", layer.description);
    content["scheme"] = boost::lexical_cast<std::string>(layer.scheme);
    build(content, "projection", layer.projection);
    build(content["bounds"], layer.bounds);
    build(content["tiles"], layer.tiles);
    build(content["available"], layer.available);
    content["minzoom"] = layer.zoom.min;
    content["maxzoom"] = layer.zoom.max;
}

} // namespace

void save(const LayerJson &layer, std::ostream &os)
{
    Json::Value content;
    build(content, layer);
    os.precision(15);
    Json::write(os, content);
}
