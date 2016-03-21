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

#include "vts-libs/registry/po.hpp"

#include "./error.hpp"
#include "./resourcebackend.hpp"
#include "./http.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace vr = vadstena::registry;

class Daemon : public service::Service {
public:
    Daemon()
        : service::Service("mapproxy", BUILD_TARGET_VERSION
                           , service::ENABLE_CONFIG_UNRECOGNIZED_OPTIONS
                           | service::ENABLE_UNRECOGNIZED_OPTIONS)
        , httpListen_(3070)
        , httpThreadCount_(5)
    {}

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    service::UnrecognizedParser::optional
    configure(const po::variables_map &vars, const std::vector<std::string>&);

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

    utility::TcpEndpoint httpListen_;
    unsigned int httpThreadCount_;
    ResourceBackend::TypedConfig resourceBackendConfig_;

    ResourceBackend::pointer resourceBackend_;

    boost::optional<Http> http_;
};

void Daemon::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    vr::registryConfiguration(cmdline, vr::defaultPath());

    config.add_options()
        ("http.listen", po::value(&httpListen_)
         ->default_value(httpListen_)->required()
         , "TCP endpoint where to listen at.")
        ("http.threadCount", po::value(&httpThreadCount_)
         ->default_value(httpThreadCount_)->required()
         , "Number of processing threads.")

        ("resource-backend.type"
         , po::value(&resourceBackendConfig_.type)->required()
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
    const auto RBType(RBPrefixDotted + "type");

    if (!vars.count(RBType)) { return {}; }

    try {
        // fetch backend type
        resourceBackendConfig_.type = vars[RBType].as<std::string>();
        // and configure
        return ResourceBackend::configure
            (RBPrefixDotted, resourceBackendConfig_);
    } catch (const UnknownResourceBackend&) {
        throw po::validation_error
            (po::validation_error::invalid_option_value, RBType);
    }
}

void Daemon::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    LOG(info3, log_)
        << "Config:"
        << "\n\thttp.listen = " << httpListen_
        << "\n\thttp.threadCount = " << httpThreadCount_
        << "\n"
        << utility::LManip([&](std::ostream &os) {
                ResourceBackend::printConfig(os, "\t" + RBPrefixDotted
                                             , resourceBackendConfig_);
            })
        ;
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
        ResourceBackend::TypedConfig config(what.substr(RBHelpPrefix.size()));
        auto parser(ResourceBackend::configure(RBPrefixDotted, config));
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

    resourceBackend_ = ResourceBackend::create(resourceBackendConfig_);
    http_ = boost::in_place(httpListen_, httpThreadCount_);

    return guard;
}

void Daemon::cleanup()
{
    // TODO: destroy stuff here
    http_ = boost::none;
    resourceBackend_.reset();
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
