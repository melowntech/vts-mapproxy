#include <fstream>

#include <boost/program_options.hpp>

#include "dbglog/dbglog.hpp"

#include "./demconfig.hpp"

namespace po = boost::program_options;

DemConfig loadDemConfig(const boost::filesystem::path &path
                        , bool ignoreMissing)
{
    po::options_description od;

    od.add_options()
        ("dem.effectiveGSD", po::value<double>()
         , "Dataset's effective GSD (in meters per pixel).")
        ;

    po::variables_map vm;

    std::ifstream f;
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try {
        LOG(info4) << "Loading " << path << ".";
        f.open(path.c_str());
        f.exceptions(std::ifstream::badbit);
        store(po::parse_config_file(f, od), vm);
        f.close();
    } catch (const std::ios_base::failure &e) {
        if (ignoreMissing) { return {}; }

        LOGTHROW(err1, std::runtime_error)
            << "Cannot load dem config file " << path << ": <" << e.what()
            << ">.";
    }

    DemConfig config;
    if (vm.count("dem.effectiveGSD")) {
        config.effectiveGSD = vm["dem.effectiveGSD"].as<double>();
    }

    return config;
}
