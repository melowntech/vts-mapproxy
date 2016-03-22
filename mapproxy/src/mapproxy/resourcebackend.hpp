#ifndef mapproxy_resourcebackend_hpp_included_
#define mapproxy_resourcebackend_hpp_included_

#include <memory>
#include <string>
#include <iostream>

#include <boost/noncopyable.hpp>
#include <boost/any.hpp>
#include <boost/program_options.hpp>

#include "service/program.hpp"

#include "./resources.hpp"

class ResourceBackend : boost::noncopyable {
public:
    typedef std::shared_ptr<ResourceBackend> pointer;

    virtual ~ResourceBackend() {}

    Resource::Groups load() const;

    class TypedConfig {
    public:
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

    static pointer create(const TypedConfig &config);

    /** Processes configuration for resource backend as an unrecognized parser.
     *
     * \param type type of resource backend
     * \param prefix configuration file section
     * \param config placeholder for configuration (i.e. parsed data)
     */
    static service::UnrecognizedParser::optional
    configure(const std::string &prefix, TypedConfig &config);

    static void printConfig(std::ostream &os, const std::string &prefix
                            , const TypedConfig &config);

    static std::vector<std::string> listTypes(const std::string &prefix = "");

    struct Factory;

    static void registerType(const std::string &type
                             , const std::shared_ptr<Factory> &factory);

protected:
    ResourceBackend() {}

    virtual Resource::Groups load_impl() const = 0;
};

inline Resource::Groups ResourceBackend::load() const
{
    return load_impl();
}

// inlines

#endif // mapproxy_resourcebackend_hpp_included_
