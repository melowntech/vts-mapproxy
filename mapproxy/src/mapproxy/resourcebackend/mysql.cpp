#include "dbglog/dbglog.hpp"

#include "utility/premain.hpp"

#include "./mysql.hpp"
#include "./factory.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace resource_backend {

namespace {

struct Factory : ResourceBackend::Factory {
    virtual ResourceBackend::pointer create(const TypedConfig &config)
    {
        return std::make_shared<Mysql>(config.value<Mysql::Config>());
    }

    virtual service::UnrecognizedParser::optional
    configure(const std::string &prefix, TypedConfig &typedConfig)
    {
        auto &config(typedConfig.assign<Mysql::Config>());

        service::UnrecognizedParser parser
            ("resource backend " + typedConfig.type + ": "
             "mysql-based resource backend");
        config.dbParams.configuration(prefix, parser.options);
        parser.configure = [&config,prefix](const po::variables_map &var)
        {
            config.dbParams.configure(prefix, var);
        };

        return parser;
    }

    void printConfig(std::ostream &os, const std::string &prefix
                     , const TypedConfig &typedConfig)
    {
        const auto &config(typedConfig.value<Mysql::Config>());

        config.dbParams.dump(os, prefix);
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    ResourceBackend::registerType("mysql", std::make_shared<Factory>());
});

} // namespace

Mysql::Mysql(const Config &config)
    : config_(config)
{
    // TODO: check validity
}

Resource::Groups Mysql::load_impl() const
{
    return {};
}

} // namespace resource_backend
