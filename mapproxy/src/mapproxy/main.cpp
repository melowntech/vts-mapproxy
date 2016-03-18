#include <cstdlib>
#include <utility>
#include <functional>
#include <map>

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "utility/streams.hpp"
#include "utility/tcpendpoint-io.hpp"
#include "service/service.hpp"

#include "./error.hpp"
#include "./resourcebackend.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

class Daemon : public service::Service {
public:
    Daemon()
        : service::Service("mapproxy", BUILD_TARGET_VERSION
                           , service::ENABLE_CONFIG_UNRECOGNIZED_OPTIONS
                           | service::ENABLE_UNRECOGNIZED_OPTIONS)
        , listen_(3070)
    {}

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    service::UnrecognizedParser::optional
    configure(const po::variables_map &vars
              , const std::vector<std::string> &unrecognized);

    void configure(const po::variables_map &vars);

    std::vector<std::string> listHelps() const;

    bool help(std::ostream &out, const std::string &what) const;

    bool prePersonaSwitch();

    Service::Cleanup start();

    int run();

    void stat(std::ostream &os);

    void cleanup();

    struct Stopper {
        Stopper(Daemon &d) : d(d) { }
        ~Stopper() { d.cleanup(); }
        Daemon &d;
    };
    friend struct Stopper;

    utility::TcpEndpoint listen_;
    std::string resourceBackendType_;
    boost::any resourceBackendConfig_;

    ResourceBackend::pointer resourceBackend_;
};

void Daemon::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    config.add_options()
        ("http.listen", po::value(&listen_)
         ->default_value(listen_)->required()
         , "TCP endpoint where to listen at.")

        ("resource-backend.type"
         , po::value(&resourceBackendType_)->required()
         , ("Resource backend type, possible values: "
            + boost::lexical_cast<std::string>
            (utility::join(ResourceBackend::listTypes(), ", "))
            + ".").c_str())
        ;

    (void) cmdline;
    (void) pd;
}

const std::string RBPrefix("resource-backend");
const std::string RBPrefixDotted(RBPrefix + ".");

service::UnrecognizedParser::optional
Daemon::configure(const po::variables_map &vars
                  , const std::vector<std::string>&)
{
    // configure resource backend
    if (!vars.count(RBPrefixDotted + "type")) { return {}; }

    try {
        return ResourceBackend::configure
            (RBPrefixDotted
             , vars[RBPrefixDotted + "type"].as<std::string>()
             , resourceBackendConfig_);
    } catch (const UnknownResourceBackend&) {
        throw po::validation_error
            (po::validation_error::invalid_option_value
             , RBPrefixDotted + "type");
    }
}

void Daemon::configure(const po::variables_map &vars)
{
    LOG(info3, log_)
        << "Config:"
        << "\n\thttp.listen = " << listen_
        << "\n"
        << utility::LManip([&](std::ostream &os) {
                ResourceBackend::printConfig(os, "\t" + RBPrefixDotted
                                             , resourceBackendType_
                                             , resourceBackendConfig_);
            })
        ;

    (void) vars;
}

const std::string RBHelpPrefix(RBPrefix + "-");

std::vector<std::string> Daemon::listHelps() const
{
    return ResourceBackend::listTypes(RBHelpPrefix);
}

bool Daemon::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("mapproxy daemon\n"
                "\n"
                );

        return true;
    }

    // check for resource backend snippet help
    if (ba::starts_with(what, RBHelpPrefix)) {
        const auto type(what.substr(RBHelpPrefix.size()));
        boost::any dummy;
        auto parser(ResourceBackend::configure(RBPrefixDotted, type, dummy));
        if (parser) {
            out << parser->options;
            return true;
        }
    }

    return false;
}

bool Daemon::prePersonaSwitch()
{
    return false; // no need to keep saved persona
}

service::Service::Cleanup Daemon::start()
{
    auto guard(std::make_shared<Stopper>(*this));

    return guard;
}

void Daemon::cleanup()
{
    // TODO: destroy stuff here
}

void Daemon::stat(std::ostream &os)
{
    os << "TODO: report stat here";
}

int Daemon::run()
{
    while (Service::isRunning()) {
        // TODO: implement me
        ::sleep(1);
    }

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    return Daemon()(argc, argv);
}
