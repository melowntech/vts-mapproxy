#include "dbglog/dbglog.hpp"

#include "utility/premain.hpp"

#include "./conffile.hpp"
#include "./factory.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace resource_backend {

namespace {

struct Factory : ResourceBackend::Factory {
    virtual ResourceBackend::pointer create(const boost::any &config)
    {
        return std::make_shared<Conffile>
            (boost::any_cast<Conffile::Config>(config));
    }

    virtual service::UnrecognizedParser::optional
    configure(const std::string &prefix, boost::any &anyConfig)
    {
        auto &config(boost::any_cast<Conffile::Config&>
                     (anyConfig = Conffile::Config()));

        service::UnrecognizedParser parser
            ("resource backend conffile: "
             "configuration file-based resource backend");
        parser.options.add_options()
            ((prefix + "path").c_str()
             , po::value(&config.path)->required()
             , "Resource backend configuration file (JSON).");

        return parser;
    }

    void printConfig(std::ostream &os, const std::string &prefix
                     , const boost::any &anyConfig)
    {
        const auto &config
            (boost::any_cast<const Conffile::Config&>(anyConfig));

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
