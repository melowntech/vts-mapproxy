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
