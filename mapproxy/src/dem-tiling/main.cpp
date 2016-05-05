#include <cstdlib>
#include <utility>
#include <functional>
#include <map>

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/thread.hpp>

#include "utility/streams.hpp"
#include "utility/tcpendpoint-io.hpp"
#include "utility/buildsys.hpp"
#include "service/cmdline.hpp"

#include "geo/geodataset.hpp"

#include "gdal-drivers/register.hpp"

#include "vts-libs/registry/po.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/subtrees.hpp"
#include "vts-libs/vts/tileindex.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace vts = vadstena::vts;

namespace vr = vadstena::registry;

class DemTiling : public service::Cmdline {
public:
    DemTiling()
        : service::Cmdline("dem-tiling", BUILD_TARGET_VERSION)
        , extentsSampling_(20)
    {
    }

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    std::string dataset_;
    fs::path output_;
    std::string referenceFrame_;
    vts::Lod lod_;
    int extentsSampling_;
};

void DemTiling::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    vr::registryConfiguration(config, vr::defaultPath());

    config.add_options()
        ("dataset", po::value(&dataset_)->required()
         , "Path to dataset to proces.")
        ("output", po::value(&output_)->required()
         , "Path output tiling file.")
        ("referenceFrame", po::value(&referenceFrame_)->required()
         , "Tiling reference frame.")
        ("lod", po::value(&lod_)->required()
         , "Level of detail at which to generate tiles.")
        ("extentsSampling", po::value(&extentsSampling_)
         ->default_value(extentsSampling_)
         , "Nuber of squares to break extents into when converting "
         "to another SRS.")
        ;

    pd.add("dataset", 1)
        .add("output", 1)
        ;

    (void) cmdline;
}

void DemTiling::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    LOG(info3, log_)
        << "Config:"
        << "\n\tdataset = " << dataset_
        << "\n\toutput = " << output_
        << "\n\treferenceFrame = " << referenceFrame_
        << "\n\tlod = " << lod_
        << "\n"
        ;
}

bool DemTiling::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("dem-tiling tool\n"
                "    Analyzes input dataset and generates tiling "
                "information.\n"
                "\n"
                );

        return true;
    }

    return false;
}

void buildSubtree(vts::TileIndex &ti, const vts::NodeInfo &root
                  , vts::Lod lod, const vts::TileRange &tileRange)
{
    LOG(info3)
        << "Processing subtree under " << root.nodeId()
        << ": tile range at lod <" << lod << ">: "
        << tileRange << ".";

    (void) ti;
    (void) root;
    (void) lod;
    (void) tileRange;
}

int DemTiling::run()
{
    auto rf(vr::Registry::referenceFrame(referenceFrame_));

    auto ds(geo::GeoDataset::open(dataset_));

    auto st(vts::findSubtrees(rf, lod_, ds.srs(), ds.extents()
                              , extentsSampling_));

    vts::TileIndex ti;
    for (const auto &range : st.ranges) {
        buildSubtree(ti, vts::NodeInfo(rf, range.first)
                     , lod_, range.second);
    }

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    return DemTiling()(argc, argv);
}
