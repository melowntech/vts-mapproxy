#ifndef mapproxy_resourcebackend_hpp_included_
#define mapproxy_resourcebackend_hpp_included_

#include <memory>
#include <string>
#include <iostream>

#include <boost/any.hpp>
#include <boost/program_options.hpp>

#include "service/program.hpp"

#include "./resources.hpp"

class ResourceBackend {
public:
    typedef std::shared_ptr<ResourceBackend> pointer;

    virtual ~ResourceBackend() {}

    Resource::Groups load() const;

    static pointer create(const std::string &type, const boost::any &config);

    /** Processes configuration for resource backend as an unrecognized parser.
     *
     * \param type type of resource backend
     * \param prefix configuration file section
     * \param config placeholder for configuration (i.e. parsed data)
     */
    static service::UnrecognizedParser::optional
    configure(const std::string &prefix, const std::string &type
              , boost::any &config);

    static void printConfig(std::ostream &os, const std::string &prefix
                            , const std::string &type
                            , const boost::any &config);

    static std::vector<std::string> listTypes(const std::string &prefix = "");

    struct Factory;

    static void registerType(const std::string &type
                             , const std::shared_ptr<Factory> &factory);

protected:
    ResourceBackend() {}

    virtual Resource::Groups load_impl() const = 0;
};

// inlines

// just forward
inline Resource::Groups ResourceBackend::load() const
{
    return load_impl();
}

#endif // mapproxy_resourcebackend_hpp_included_
