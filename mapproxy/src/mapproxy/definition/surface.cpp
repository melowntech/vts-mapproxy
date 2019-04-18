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

#include "surface.hpp"

namespace vr = vtslibs::registry;

namespace resource {

constexpr Resource::Generator::Type Surface::type;

bool Surface::Introspection::empty() const
{
    return (tms.empty() && geodata.empty()
            && !position && browserOptions.empty());
}

bool Surface::Introspection::operator!=(const Introspection &other) const
{
    // introspection can safely change
    if (tms != other.tms) { return true; }
    if (geodata != other.geodata) { return true; }
    if (position != other.position) { return true; }

    if (browserOptions.empty() != other.browserOptions.empty()) {
        return true;
    }
    if (!browserOptions.empty()
        && (boost::any_cast<const Json::Value&>(browserOptions)
            != boost::any_cast<const Json::Value&>(other.browserOptions)))
    {
        return true;
    }

    return false;
}

void Surface::parse(const Json::Value &value)
{
    if (value.isMember("nominalTexelSize")) {
        nominalTexelSize = boost::in_place();
        Json::get(*nominalTexelSize, value, "nominalTexelSize");
    }

    if (value.isMember("mergeBottomLod")) {
        mergeBottomLod = boost::in_place();
        Json::get(*mergeBottomLod, value, "mergeBottomLod");
    }

    heightFunction = HeightFunction::parse(value, "heightFunction");

    if (value.isMember("introspection")) {
        const auto &jintrospection(value["introspection"]);

        introspection.tms
            = introspection::layersFrom(jintrospection, "tms");
        introspection.geodata
            = introspection::layersFrom(jintrospection, "geodata");

        if (jintrospection.isMember("position")) {
            introspection.position
                = vr::positionFromJson(jintrospection["position"]);
        }

        if (jintrospection.isMember("browserOptions")) {
            introspection.browserOptions
                = Json::check(jintrospection["browserOptions"]
                              , Json::objectValue);
        }
    }
}

void Surface::build(Json::Value &value) const
{
    if (nominalTexelSize) {
        value["nominalTexelSize"] = *nominalTexelSize;
    }
    if (mergeBottomLod) {
        value["mergeBottomLod"] = *mergeBottomLod;
    }

    if (heightFunction) {
        auto &tmp(value["heightFunction"] = Json::objectValue);
        heightFunction->build(tmp);
    }

    if (!introspection.empty()) {
        auto &jintrospection(value["introspection"] = Json::objectValue);
        introspection::layersTo(jintrospection, "tms", introspection.tms) ;
        introspection::layersTo
            (jintrospection, "geodata", introspection.geodata);

        if (introspection.position) {
            jintrospection["position"] = vr::asJson(*introspection.position);
        }

        if (!introspection.browserOptions.empty()) {
            jintrospection["browserOptions"]
                = boost::any_cast<const Json::Value&>
                (introspection.browserOptions);
        }
    }
}

Changed Surface::changed_impl(const DefinitionBase &o)
    const
{
    const auto &other(o.as<Surface>());

    // manually set data can be changed safely
    if (nominalTexelSize != other.nominalTexelSize) {
        return Changed::safely;
    }

    if (mergeBottomLod != other.mergeBottomLod) {
        return Changed::safely;
    }

    if (introspection != other.introspection) {
        return Changed::safely;
    }

    if (HeightFunction::changed(heightFunction, other.heightFunction)) {
        return Changed::yes;
    }

    return Changed::no;
}

} // namespace resource
