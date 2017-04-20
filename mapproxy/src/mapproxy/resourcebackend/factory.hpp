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

#ifndef mapproxy_resourcebackend_factory_hpp_included_
#define mapproxy_resourcebackend_factory_hpp_included_

#include "../resourcebackend.hpp"

struct ResourceBackend::Factory {
    typedef std::shared_ptr<Factory> pointer;
    typedef ResourceBackend::GenericConfig GenericConfig;
    typedef ResourceBackend::TypedConfig TypedConfig;

    virtual ~Factory() {}
    virtual ResourceBackend::pointer create(const GenericConfig &genericConfig
                                            , const TypedConfig &config) = 0;

    virtual service::UnrecognizedParser::optional
    configure(const std::string &prefix, TypedConfig &config
              , const service::UnrecognizedOptions &unrecognized) = 0;

    virtual void printConfig(std::ostream &os, const std::string &prefix
                             , const TypedConfig &config) = 0;
};

#endif // mapproxy_resourcebackend_factory_hpp_included_
