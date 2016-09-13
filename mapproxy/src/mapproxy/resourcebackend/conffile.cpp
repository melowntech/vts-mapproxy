#include "dbglog/dbglog.hpp"

#include "utility/premain.hpp"
#include "service/program.hpp"

#include "../error.hpp"
#include "./conffile.hpp"
#include "./factory.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace resource_backend {

namespace {

struct Factory : ResourceBackend::Factory {
    virtual ResourceBackend::pointer create(const GenericConfig &genericConfig
                                            , const TypedConfig &config)
    {
        return std::make_shared<Conffile>
            (genericConfig, config.value<Conffile::Config>());
    }

    virtual service::UnrecognizedParser::optional
    configure(const std::string &prefix, TypedConfig &typedConfig
              , const service::UnrecognizedOptions&)
    {
        auto &config(typedConfig.assign<Conffile::Config>());

        service::UnrecognizedParser parser
            ("resource backend " + typedConfig.type + ": "
             "configuration file-based resource backend");
        parser.options.add_options()
            ((prefix + "path").c_str()
             , po::value(&config.path)->required()
             , "Path to resource file (JSON).");

        return parser;
    }

    void printConfig(std::ostream &os, const std::string &prefix
                     , const TypedConfig &typedConfig)
    {
        const auto &config(typedConfig.value<Conffile::Config>());

        os << prefix << "path = " << config.path << "\n";
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    ResourceBackend::registerType("conffile", std::make_shared<Factory>());
});

} // namespace

Conffile::Conffile(const GenericConfig &genericConfig
                   , const Config &config)
    : ResourceBackend(genericConfig), config_(config)
{
    // try to load config file now
    load_impl();
}

Resource::map Conffile::load_impl() const
{
    return loadResources(config_.path, {}, genericConfig_.fileClassSettings);
}

} // namespace resource_backend
