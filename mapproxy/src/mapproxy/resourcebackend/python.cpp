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
    virtual ResourceBackend::pointer create(const TypedConfig &config)
    {
        std::call_once(onceFlag, [&]()
        {
            // Initialize python interpreter
            ::Py_Initialize();

            // make sure dbglog is ready
            dbglog::py::import();
        });

        return std::make_shared<Python>(config.value<Python::Config>());
    }

    virtual service::UnrecognizedParser::optional
    configure(const std::string &prefix, TypedConfig &typedConfig)
    {
        auto &config(typedConfig.assign<Python::Config>());

        service::UnrecognizedParser parser
            ("resource backend " + typedConfig.type + ": "
             "mysql-based resource backend");

        parser.options.add_options()
            ((prefix + "script").c_str()
             , po::value(&config.script)->required()
             , "Path pythong script. It must privide global function run().");

        return parser;
    }

    void printConfig(std::ostream &os, const std::string &prefix
                     , const TypedConfig &typedConfig)
    {
        const auto &config(typedConfig.value<Python::Config>());

        os << prefix << "script = " << config.script << "\n";
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    ResourceBackend::registerType("python", std::make_shared<Factory>());
});

} // namespace

Python::Python(const Config &config)
    : run_()
{
    try {
        run_ = pysupport::import(config.script).attr("run");
    } catch (const python::error_already_set&) {
        // TODO: handle exceptions
        ::PyErr_Print();
        LOGTHROW(err2, Error)
            << "Run importing python script from " << config.script << ".";
    }
}

Resource::map Python::load_impl() const
{
    try {
        return loadResourcesFromPython(python::list(run_()));
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

} // namespace resource_backend
