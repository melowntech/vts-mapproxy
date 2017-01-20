#include <cstdlib>
#include <utility>
#include <functional>
#include <map>

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include "cpl_minixml.h"

#include "utility/streams.hpp"
#include "utility/buildsys.hpp"
#include "utility/openmp.hpp"
#include "utility/raise.hpp"
#include "utility/duration.hpp"
#include "utility/time.hpp"
#include "service/cmdline.hpp"

#include "geo/geodataset.hpp"
#include "geo/gdal.hpp"

#include "gdal-drivers/register.hpp"
#include "gdal-drivers/solid.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

class Calipers : public service::Cmdline {
public:
    Calipers()
        : service::Cmdline("calipers", BUILD_TARGET_VERSION)
    {
    }

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    fs::path dataset_;
    std::string referenceFrame_;
};

void Calipers::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("dataset", po::value(&dataset_)->required()
         , "Path to GDAL dataset to examine.")
        ("referenceFrame", po::value(&referenceFrame_)->required()
         , "Reference frame.")
        ;

    pd.add("dataset", 1)
        ;

    (void) config;
}

void Calipers::configure(const po::variables_map &vars)
{
    dataset_ = fs::absolute(dataset_);

    LOG(info3, log_)
        << "Config:"
        << "\n\tdataset = " << dataset_
        << "\n\treferenceFrame = " << referenceFrame_
        << "\n"
        ;

    (void) vars;
}

bool Calipers::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("calipers dataset [options]\n"
                "    Examines GDAL dataset.\n"
                "\n"
                );

        return true;
    }

    return false;
}

int Calipers::run()
{
    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    return Calipers()(argc, argv);
}
