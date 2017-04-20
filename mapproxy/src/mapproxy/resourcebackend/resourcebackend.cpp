/**
 * Copyright (c) 2017 Melown Technologies SE
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

#include "dbglog/dbglog.hpp"

#include "../error.hpp"
#include "../resourcebackend.hpp"
#include "./factory.hpp"

namespace po = boost::program_options;

namespace {

typedef std::map<std::string, ResourceBackend::Factory::pointer> Registry;
Registry registry;

ResourceBackend::Factory::pointer findFactory(const std::string &type)
{
    auto fregistry(registry.find(type));
    if (fregistry == registry.end()) {
        LOGTHROW(err1, UnknownResourceBackend)
            << "Unknown resource backend <" << type << ">.";
    }
    return fregistry->second;
}

} // namespace

void ResourceBackend::registerType(const std::string &type
                                   , const Factory::pointer &factory)
{
    registry.insert(Registry::value_type(type, factory));
}

service::UnrecognizedParser::optional
ResourceBackend::configure(const std::string &prefix, TypedConfig &config
                           , const service::UnrecognizedOptions &unrecognized)
{
    return findFactory(config.type)->configure(prefix, config, unrecognized);
}

void ResourceBackend::printConfig(std::ostream &os, const std::string &prefix
                                  , const TypedConfig &config)
{
    os << prefix << "type = " << config.type << "\n";
    return findFactory(config.type)->printConfig(os, prefix, config);
}

std::vector<std::string> ResourceBackend::listTypes(const std::string &prefix)
{
    std::vector<std::string> out;
    for (const auto &ritem : registry) {
        out.push_back(prefix + ritem.first);
    }
    return out;
}

ResourceBackend::pointer
ResourceBackend::create(const GenericConfig &genericConfig
                        , const TypedConfig &config)
{
    try {
        return findFactory(config.type)->create(genericConfig, config);
    } catch (const boost::bad_any_cast&) {
        LOGTHROW(err2, InvalidConfiguration)
            << "Passed configuration does not match resource backend <"
            << config.type << ">.";
    }
    throw;
}
