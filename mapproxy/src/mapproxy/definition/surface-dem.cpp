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

#include "../support/python.hpp"

#include "./surface.hpp"

namespace resource {

namespace {

void parseDefinition(SurfaceDem &def, const Json::Value &value)
{
    Json::get(def.dem.dataset, value, "dataset");
    if (value.isMember("mask")) {
        std::string s;
        Json::get(s, value, "mask");
        def.mask = s;;
    }

    if (value.isMember("textureLayerId")) {
        Json::get(def.textureLayerId, value, "textureLayerId");
    }

    if (value.isMember("geoidGrid")) {
        std::string s;
        Json::get(s, value, "geoidGrid");
        def.dem.geoidGrid = s;
    }

    if (value.isMember("heightcodingAlias")) {
        def.heightcodingAlias = boost::in_place();
        Json::get(*def.heightcodingAlias, value, "heightcodingAlias");
    }

    def.parse(value);
}

void buildDefinition(Json::Value &value, const SurfaceDem &def)
{
    value["dataset"] = def.dem.dataset;
    if (def.mask) {
        value["mask"] = def.mask->string();
    }

    if (def.textureLayerId) {
        value["textureLayerId"] = def.textureLayerId;
    }
    if (def.dem.geoidGrid) {
        value["geoidGrid"] = *def.dem.geoidGrid;
    }
    if (def.heightcodingAlias) {
        value["heightcodingAlias"] = *def.heightcodingAlias;
    }

    def.build(value);
}

void parseDefinition(SurfaceDem &def
                     , const boost::python::dict &value)
{
    namespace python = boost::python;

    def.dem.dataset = py2utf8(value["dataset"]);

    if (value.has_key("mask")) {
        def.mask = py2utf8(value["mask"]);
    }

    if (value.has_key("textureLayerId")) {
        def.textureLayerId = python::extract<int>(value["textureLayerId"]);
    }

    if (value.has_key("geoidGrid")) {
        def.dem.geoidGrid = py2utf8(value["geoidGrid"]);
    }

    if (value.has_key("heightcodingAlias")) {
        def.heightcodingAlias = py2utf8(value["heightcodingAlias"]);
    }

    def.parse(value);
}

} // namespace

void SurfaceDem::from_impl(const boost::any &value)
{
    if (const auto *json = boost::any_cast<Json::Value>(&value)) {
        parseDefinition(*this, *json);
    } else if (const auto *py
               = boost::any_cast<boost::python::dict>(&value))
    {
        parseDefinition(*this, *py);
    } else {
        LOGTHROW(err1, Error)
            << "SurfaceDem: Unsupported configuration from: <"
            << value.type().name() << ">.";
    }
}

void SurfaceDem::to_impl(boost::any &value) const
{
    if (auto *json = boost::any_cast<Json::Value>(&value)) {
        buildDefinition(*json, *this);
    } else {
        LOGTHROW(err1, Error)
            << "SurfaceDem:: Unsupported serialization into: <"
            << value.type().name() << ">.";
    }
}

Changed SurfaceDem::changed_impl(const DefinitionBase &o) const
{
    const auto &other(o.as<SurfaceDem>());

    if (dem != other.dem) { return Changed::yes; }
    if (mask != other.mask) { return Changed::yes; }
    if (textureLayerId != other.textureLayerId) { return Changed::yes; }

    return Surface::changed_impl(o);
}

} // namespace resource
