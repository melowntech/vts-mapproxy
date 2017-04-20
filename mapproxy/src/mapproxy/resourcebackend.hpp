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

#ifndef mapproxy_resourcebackend_hpp_included_
#define mapproxy_resourcebackend_hpp_included_

#include <memory>
#include <string>
#include <iostream>

#include <boost/noncopyable.hpp>
#include <boost/any.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>

#include "service/program.hpp"

#include "./resource.hpp"

class ResourceBackend : boost::noncopyable {
public:
    typedef std::shared_ptr<ResourceBackend> pointer;

    virtual ~ResourceBackend() {}

    Resource::map load() const;

    void error(const Resource::Id &resourceId, const std::string &message)
        const;

    struct GenericConfig {
        FileClassSettings fileClassSettings;
    };

    class TypedConfig {
    public:
        /** Resource backend type.
         */
        std::string type;

        template <typename T>
        T& value() { return boost::any_cast<T&>(value_); }
        template <typename T>
        const T& value() const { return boost::any_cast<const T&>(value_); }

        template <typename T>
        T& assign(const T &value = T()) {
            return boost::any_cast<T&>(value_ = value);
        }

        TypedConfig(const std::string &type = "") : type(type) {}

    private:
        boost::any value_;
    };

    const GenericConfig& genericConfig() const { return genericConfig_; }

    static pointer create(const GenericConfig &genericConfig
                          , const TypedConfig &config);

    /** Processes configuration for resource backend as an unrecognized parser.
     *
     * \param type type of resource backend
     * \param prefix configuration file section
     * \param config placeholder for configuration (i.e. parsed data)
     */
    static service::UnrecognizedParser::optional
    configure(const std::string &prefix, TypedConfig &config
              , const service::UnrecognizedOptions &unrecognized);

    static void printConfig(std::ostream &os, const std::string &prefix
                            , const TypedConfig &config);

    static std::vector<std::string> listTypes(const std::string &prefix = "");

    struct Factory;

    static void registerType(const std::string &type
                             , const std::shared_ptr<Factory> &factory);

protected:
    ResourceBackend(const GenericConfig &genericConfig)
        : genericConfig_(genericConfig)
    {}

    virtual Resource::map load_impl() const = 0;

    virtual void error_impl(const Resource::Id&, const std::string&) const {}

    GenericConfig genericConfig_;
};

inline Resource::map ResourceBackend::load() const
{
    return load_impl();
}

inline void ResourceBackend::error(const Resource::Id &resourceId
                                   , const std::string &message) const
{
    return error_impl(resourceId, message);
}

// inlines

#endif // mapproxy_resourcebackend_hpp_included_
