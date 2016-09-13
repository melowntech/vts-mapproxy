#include <thread>

#include "dbglog/dbglog.hpp"

#include "utility/premain.hpp"

#include "pydbglog/dbglogmodule.hpp"
#include "pysupport/import.hpp"

#include "../error.hpp"
#include "./python.hpp"
#include "./factory.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace python = boost::python;

namespace resource_backend {

namespace {

std::once_flag onceFlag;

struct Factory : ResourceBackend::Factory {
    virtual ResourceBackend::pointer create(const GenericConfig &genericConfig
                                            , const TypedConfig &config)
    {
        std::call_once(onceFlag, [&]()
        {
            // Initialize python interpreter
            ::Py_Initialize();

            // make sure dbglog is ready
            dbglog::py::import();
        });

        return std::make_shared<Python>
            (genericConfig, config.value<Python::Config>());
    }

    virtual service::UnrecognizedParser::optional
    configure(const std::string &prefix, TypedConfig &typedConfig
              , const service::UnrecognizedOptions &unrecognized)
    {
        auto &config(typedConfig.assign<Python::Config>());

        service::UnrecognizedParser parser
            ("resource backend " + typedConfig.type + ": "
             "mysql-based resource backend");

        parser.options.add_options()
            ((prefix + "script").c_str()
             , po::value(&config.script)->required()
             , "Path pythong script. It must privide global function run().")
            ;

        const auto optPrefix(prefix + "option");
        const auto optPrefixDotted(optPrefix +".");
        const auto optPrefixDashed("--" + optPrefixDotted);

        auto add([&](const std::string &name) -> std::string*
        {
            auto res(config.options.insert
                     (Python::Config::Options::value_type(name, "")));
            if (!res.second) { return nullptr; }
            return &res.first->second;
        });

        for (const auto &option : unrecognized.cmdline) {
            if (option.find(optPrefixDashed) != 0) { continue; }

            auto name(option.substr(optPrefixDashed.size()));
            if (auto *location = add(name)) {
                parser.options.add_options()
                    (option.substr(2).c_str(), po::value(location)
                     , "extra option")
                    ;
            }
        }
        for (const auto &config : unrecognized.config) {
            for (const auto &options : config) {
                const auto &option(options.first);
                if (option.find(optPrefixDotted) != 0) { continue; }

                auto name(option.substr(optPrefixDotted.size()));

                if (auto *location = add(name)) {
                    parser.options.add_options()
                        (option.c_str(), po::value(location)
                     , "extra option")
                        ;
                }
            }
        }

        return parser;
    }

    void printConfig(std::ostream &os, const std::string &prefix
                     , const TypedConfig &typedConfig)
    {
        const auto &config(typedConfig.value<Python::Config>());

        os << prefix << "script = " << config.script << "\n";

        for (const auto &option : config.options) {
            os << prefix << "option." << option.first << " = "
               << option.second << "\n";
        }
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    ResourceBackend::registerType("python", std::make_shared<Factory>());
});

} // namespace

Python::Python(const GenericConfig &genericConfig, const Config &config)
    : ResourceBackend(genericConfig), run_(), error_()
{
    try {
        python::dict options;
        for (const auto &option : config.options) {
            options[option.first] = option.second;
        }

        run_ = pysupport::import(config.script)
            .attr("resource_backend")(options);
        if (PyObject_HasAttrString(run_.ptr(), "error")) {
            error_ = run_.attr("error");
        }
    } catch (const python::error_already_set&) {
        // TODO: handle exceptions
        ::PyErr_Print();
        LOGTHROW(err2, Error)
            << "Run importing python script from " << config.script << ".";
    }
}

Resource::map Python::load_impl() const
{
    std::unique_lock<std::mutex> lock(mutex_);
    try {
        return loadResourcesFromPython
            (python::list(run_())
             , [this](const Resource::Id &id, const std::string &error) {
                error_impl(id, error);
            }, genericConfig_.fileClassSettings);
    } catch (const python::error_already_set&) {
        // TODO: handle exceptions
        LOG(err3) << "Resource backend run failed, py exception follows:";

        // NB: Do not set system error indicators (sys.last_*) otherwise leak to
        // next exception exists -> we must call PyErr_PrintEx with false!
        ::PyErr_PrintEx(false);

        LOGTHROW(err3, Error)
            << "Run failed.";
        throw;
    }
}

void Python::error_impl(const Resource::Id &resourceId
                        , const std::string &message) const
{
    std::unique_lock<std::mutex> lock(mutex_);
    return errorRaw(resourceId, message);
}

void Python::errorRaw(const Resource::Id &resourceId
                      , const std::string &message) const
{
    if (error_) {
        try {
            error_(resourceId.referenceFrame, resourceId.group
                   , resourceId.id, message);
        } catch (const python::error_already_set&) {
            // TODO: handle exceptions
            LOG(err3) << "Resource backend error report failed, "
                "py exception follows:";

            // NB: Do not set system error indicators (sys.last_*) otherwise
            // leak to next exception exists -> we must call PyErr_PrintEx with
            // false!
            ::PyErr_PrintEx(false);
        }
    }
}

} // namespace resource_backend
