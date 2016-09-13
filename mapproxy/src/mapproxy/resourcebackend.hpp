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
