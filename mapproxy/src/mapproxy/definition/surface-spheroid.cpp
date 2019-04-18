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

#include "surface.hpp"
#include "factory.hpp"

namespace resource {

constexpr char SurfaceSpheroid::driverName[];

namespace {

utility::PreMain register_([]() { registerDefinition<SurfaceSpheroid>(); });

void parseDefinition(SurfaceSpheroid &def, const Json::Value &value)
{
    if (value.isMember("textureLayerId")) {
        Json::get(def.textureLayerId, value, "textureLayerId");
    }
    if (value.isMember("geoidGrid")) {
        std::string s;
        Json::get(s, value, "geoidGrid");
        def.geoidGrid = s;
    } else {
        def.geoidGrid = boost::none;
    }

    def.parse(value);
}

void buildDefinition(Json::Value &value, const SurfaceSpheroid &def)
{
    if (def.textureLayerId) {
        value["textureLayerId"] = def.textureLayerId;
    }
    if (def.geoidGrid) {
        value["geoidGrid"] = *def.geoidGrid;
    }

    def.build(value);
}

} // namespace

void SurfaceSpheroid::from_impl(const Json::Value &value)
{
    parseDefinition(*this, value);
}

void SurfaceSpheroid::to_impl(Json::Value &value) const
{
    buildDefinition(value, *this);
}

Changed SurfaceSpheroid::changed_impl(const DefinitionBase &o)
    const
{
    const auto &other(o.as<SurfaceSpheroid>());

    if (textureLayerId != other.textureLayerId) { return Changed::yes; }
    if (geoidGrid != other.geoidGrid) { return Changed::yes; }

    return Surface::changed_impl(o);
}

} // namespace resource
