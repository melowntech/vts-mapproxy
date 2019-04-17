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

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/registry/json.hpp"
#include "vts-libs/registry/py.hpp"

#include "../error.hpp"

#include "parse.hpp"

namespace vf = geo::vectorformat;

namespace resource {

void parse(geo::VectorFormat &format, const Json::Value &value)
{
    Json::check(value, Json::stringValue);
    try {
        format = boost::lexical_cast<geo::VectorFormat>(value);
    } catch (boost::bad_lexical_cast) {
        utility::raise<FormatError>
            ("Value stored in format is not a valid height"
             " coded data format.");
    }
}

void build(Json::Value &value, const geo::VectorFormat &format)
{
    value = boost::lexical_cast<std::string>(format);
}

void parse(vf::Config &config, geo::VectorFormat format
           , const Json::Value &value)
{
    if (!value.isObject()) {
        LOGTHROW(err1, FormatError)
            << "Geodata definition[formatConfig] is not an object.";
    }

    switch (format) {
    case geo::VectorFormat::geodataJson: {
        auto &c(createGeodataConfig(config));
        Json::getOpt(c.resolution, value, "resolution");
    } break;

    default:
        // ignore
        break;
    }
}

void build(Json::Value &value, const vf::Config &config)
{
    value = Json::objectValue;
    if (const auto *c = boost::get<vf::GeodataConfig>(&config)) {
        value["resolution"] = c->resolution;
    }
}

} // namespace resource
