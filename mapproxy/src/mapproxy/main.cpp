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
#include "utility/buildsys.hpp"
#include "service/service.hpp"

#include "gdal-drivers/register.hpp"

#include "vts-libs/registry/po.hpp"

#include "./error.hpp"
#include "./resourcebackend.hpp"
#include "./generator.hpp"
#include "./http.hpp"
#include "./core.hpp"
#include "./gdalsupport.hpp"

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
        , httpListen_(3070), httpThreadCount_(5), httpEnableBrowser_(false)
    {
        generatorsConfig_.root
            = utility::buildsys::installPath("var/mapproxy/store");
        generatorsConfig_.resourceRoot
            = utility::buildsys::installPath("var/mapproxy/datasets");
        gdalWarperOptions_.processCount = 8;
        gdalWarperOptions_.tmpRoot
            = utility::buildsys::installPath("var/mapproxy/tmp");
        generatorsConfig_.resourceUpdatePeriod = 300;
    }

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

    fs::path resourceRoot_;

    utility::TcpEndpoint httpListen_;
    unsigned int httpThreadCount_;
    bool httpEnableBrowser_;
    ResourceBackend::TypedConfig resourceBackendConfig_;
    Generators::Config generatorsConfig_;
    GdalWarper::Options gdalWarperOptions_;

    ResourceBackend::pointer resourceBackend_;
    boost::optional<GdalWarper> gdalWarper_;
    boost::optional<Generators> generators_;
    boost::optional<Core> core_;
    boost::optional<Http> http_;
};

void Daemon::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    vr::registryConfiguration(config, vr::defaultPath());

    config.add_options()
        ("store.path", po::value(&generatorsConfig_.root)
         ->default_value(generatorsConfig_.root)->required()
         , "Path to internal store.")

        ("http.listen", po::value(&httpListen_)
         ->default_value(httpListen_)->required()
         , "TCP endpoint where to listen at.")
        ("http.threadCount", po::value(&httpThreadCount_)
         ->default_value(httpThreadCount_)->required()
         , "Number of processing threads.")
        ("http.enableBrowser", po::value(&httpEnableBrowser_)
         ->default_value(httpEnableBrowser_)->required()
         , "Enables resource browsering functionaly if set to true.")

        ("gdal.processCount"
         , po::value(&gdalWarperOptions_.processCount)
         ->default_value(gdalWarperOptions_.processCount)->required())
        ("gdal.tmpRoot"
         , po::value(&gdalWarperOptions_.tmpRoot)
         ->default_value(gdalWarperOptions_.tmpRoot)->required())

        ("resource-backend.type"
         , po::value(&resourceBackendConfig_.type)->required()
         , ("Resource backend type, possible values: "
            + boost::lexical_cast<std::string>
            (utility::join(ResourceBackend::listTypes(), ", "))
            + ".").c_str())
        ("resource-backend.updatePeriod"
         , po::value(&generatorsConfig_.resourceUpdatePeriod)
         ->default_value(generatorsConfig_.resourceUpdatePeriod)->required()
         , "Update period between resource list update (in seconds).")
        ("resource-backend.root"
         , po::value(&generatorsConfig_.resourceRoot)
         ->default_value(generatorsConfig_.resourceRoot)->required()
         , "Root of datasets defined as relative path.")
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

    // prepare generators' configuration
    generatorsConfig_.root = absolute(generatorsConfig_.root);
    generatorsConfig_.resourceRoot = absolute(generatorsConfig_.resourceRoot);
    if (httpEnableBrowser_) {
        generatorsConfig_.fileFlags |= FileFlags::browserEnabled;
    }

    LOG(info3, log_)
        << "Config:"
        << "\n\tstore.path = " << generatorsConfig_.root
        << "\n\thttp.listen = " << httpListen_
        << "\n\thttp.threadCount = " << httpThreadCount_
        << "\n\tresource-backend.updatePeriod = "
        << generatorsConfig_.resourceUpdatePeriod
        << "\n\tresource-backend.root = "
        << generatorsConfig_.resourceRoot
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

    // warper must be first since it uses processes
    gdalWarper_ = boost::in_place(gdalWarperOptions_);

    resourceBackend_ = ResourceBackend::create(resourceBackendConfig_);
    generators_ = boost::in_place
        (generatorsConfig_, resourceBackend_);
    core_ = boost::in_place(std::ref(*generators_), std::ref(*gdalWarper_));
    http_ = boost::in_place(httpListen_, httpThreadCount_, std::ref(*core_));

    return guard;
}

void Daemon::cleanup()
{
    // destroy, in reverse order
    http_ = boost::none;
    core_ = boost::none;
    generators_.reset();
    resourceBackend_.reset();

    gdalWarper_ = boost::none;
}

void Daemon::stat(std::ostream &os)
{
    os << "TODO: report stat here";
}

int Daemon::run()
{
    try {
        while (Service::isRunning()) {
            gdalWarper_->housekeeping();
            ::usleep(500000);
        }
    } catch (AbandonAll) {
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    return Daemon()(argc, argv);
}
