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

#include "../support/python.hpp"

#include "./tms.hpp"

namespace resource {

constexpr Resource::Generator::Type TmsCommon::type;

void TmsCommon::parse(const Json::Value &value)
{
    if (value.isMember("options")) { options = value["options"]; }
}

void TmsCommon::build(Json::Value &value) const
{
    if (!options.empty()) {
        value["options"] = boost::any_cast<Json::Value>(options);
    }
}

void TmsCommon::parse(const boost::python::dict &value)
{
    if (value.has_key("options")) {
        LOG(warn2)
            << "Generic options not supported in python TMS configuration.";
    }
}

namespace {

bool optionsChanged(const boost::any &l, const boost::any &r)
{
    const auto *lopt(boost::any_cast<Json::Value>(&l));
    const auto *ropt(boost::any_cast<Json::Value>(&r));

    // different availability -> changed
    if (bool(lopt) != bool(ropt)) { return true; }

    if (!lopt) { return false; }

    // compare value-wise
    return *lopt != *ropt;
}

} // namespace

Changed TmsCommon::changed_impl(const DefinitionBase &o) const
{
    const auto &other(o.as<TmsCommon>());

    // options can change
    if (optionsChanged(options, other.options)) { return Changed::safely; }

    // not changed
    return Changed::no;
}

} // namespace resource
