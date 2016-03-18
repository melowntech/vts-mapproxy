#include "dbglog/dbglog.hpp"

#include "utility/premain.hpp"

#include "./conffile.hpp"
#include "./factory.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace resource_backend {

namespace {

struct Factory : ResourceBackend::Factory {
    virtual ResourceBackend::pointer create(const TypedConfig &config)
    {
        return std::make_shared<Conffile>(config.value<Conffile::Config>());
    }

    virtual service::UnrecognizedParser::optional
    configure(const std::string &prefix, TypedConfig &typedConfig)
    {
        auto &config(typedConfig.assign<Conffile::Config>());

        service::UnrecognizedParser parser
            ("resource backend " + typedConfig.type + ": "
             "configuration file-based resource backend");
        parser.options.add_options()
            ((prefix + "path").c_str()
             , po::value(&config.path)->required()
             , "Resource backend configuration file (JSON).");

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

Conffile::Conffile(const Config &config)
    : config_(config)
{
    // TODO: check validity
}

Resource::Groups Conffile::load_impl() const
{
    return {};
}

} // namespace resource_backend
