#ifndef mapproxy_resourcebackend_factory_hpp_included_
#define mapproxy_resourcebackend_factory_hpp_included_

#include "../resourcebackend.hpp"

struct ResourceBackend::Factory {
    typedef std::shared_ptr<Factory> pointer;

    virtual ~Factory() {}
    virtual ResourceBackend::pointer create(const boost::any &config) = 0;

    virtual service::UnrecognizedParser::optional
    configure(const std::string &prefix, boost::any &config) = 0;

    virtual void printConfig(std::ostream &os, const std::string &prefix
                             , const boost::any &config) = 0;
};

#endif // mapproxy_resourcebackend_factory_hpp_included_
