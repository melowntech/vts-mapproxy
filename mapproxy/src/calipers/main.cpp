/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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

#include <opencv2/highgui/highgui.hpp>

#include "cpl_minixml.h"

#include "utility/streams.hpp"
#include "utility/buildsys.hpp"
#include "utility/openmp.hpp"
#include "utility/raise.hpp"
#include "utility/duration.hpp"
#include "utility/time.hpp"
#include "utility/enum-io.hpp"

#include "math/transform.hpp"

#include "service/cmdline.hpp"

#include "geo/geodataset.hpp"
#include "geo/gdal.hpp"

#include "gdal-drivers/register.hpp"
#include "gdal-drivers/solid.hpp"

#include "vts-libs/registry.hpp"
#include "vts-libs/registry/po.hpp"
#include "vts-libs/registry/io.hpp"
#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/csconvertor.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/basetypes.hpp"
#include "vts-libs/vts/math.hpp"
#include "vts-libs/vts/io.hpp"

#include "./calipers.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace ublas = boost::numeric::ublas;
namespace vr = vtslibs::registry;
namespace vts = vtslibs::vts;

class Calipers : public service::Cmdline {
public:
    Calipers()
        : service::Cmdline("mapproxy-calipers", BUILD_TARGET_VERSION)
    {}

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    fs::path dataset_;
    std::string referenceFrameId_;
    calipers::Config config_;
};

void Calipers::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    vr::registryConfiguration(cmdline, vr::defaultPath());

    cmdline.add_options()
        ("dataset", po::value(&dataset_)->required()
         , "Path to GDAL dataset to examine.")
        ("referenceFrame", po::value(&referenceFrameId_)->required()
         , "Reference frame.")
        ("datasetType", po::value<calipers::DatasetType>()
         , "Dataset type (dem or ophoto). Mandatory only "
         "if autodetect fails.")

        ("demToOphotoScale", po::value(&config_.demToOphotoScale)
         ->default_value(config_.demToOphotoScale)->required()
         , "Inverse scale between DEM's resolution and resolution of "
         "most detailed orthophoto that can be draped on it. "
         "Used for bottom LOD calculation. "
         "To get 2x better ophoto (i.e. resolution scale 1/2) use 2.")

        ("tileFractionLimit", po::value(&config_.tileFractionLimit)
         ->default_value(config_.tileFractionLimit)->required()
         , "Fraction of tile when rastrization algorithm stops."
         "Inverse value, 4 means 1/4 of tile.")
        ;

    pd.add("dataset", 1)
        .add("referenceFrame", 1)
        ;

    (void) config;
}

void Calipers::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    dataset_ = fs::absolute(dataset_);

    if (vars.count("datasetType")) {
        config_.datasetType = vars["datasetType"].as<calipers::DatasetType>();
    }

    LOG(info3, log_)
        << "Config:"
        << "\n\tdataset = " << dataset_
        << "\n\treferenceFrame = " << referenceFrameId_
        << "\n"
        ;

    (void) vars;
}

bool Calipers::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("mapproxy-calipers dataset referenceFrame [options]\n"
                "    Measures GDAL dataset in given reference frame.\n"
                "\n"
                "    Output format (stdout, machine readable):\n"
                "\n"
                "        gsd: GSD\n"
                "        range<SRS1>: lodRange lod/tileRange\n"
                "        range<SRS2>: lodRange lod/tileRange\n"
                "        ...\n"
                "        range<SRSN>: lodRange lod/tileRange\n"
                "        range:       lodRange tileRange\n"
                "        position: VTS-position\n"
                "\n"
                "    Where\n"
                "        GSD       is computed ground sample distance\n"
                "                  (resolution in meters per pixel)\n"
                "        SRS1-N    SRS in spatial division node 1-N\n"
                "        lodRange  estimated LOD range\n"
                "        lod/tileRange measured range at given LOD\n"
                "        tileRange measured tile range at minimal LOD\n"
                "        position  best estimated position\n"
                "\n"
                "    NB:\n"
                "        * values from range<SRS*> lines are to be fed to\n"
                "          mapproxy-tiling tool\n"
                "        * values from range line are be put in mapproxy\n"
                "          resource configuration\n"
                "\n"
                );

        return true;
    }

    return false;
}


int Calipers::run()
{
    const auto ds(geo::GeoDataset::open(dataset_).descriptor());

    auto m(calipers::measure(vr::system.referenceFrames(referenceFrameId_)
                             , ds, config_));

    if (m.nodes.empty()) {
        // not feasible
        return 1;
    }

    std::cout << "gsd: " << gsd << "\n";

    for (const auto &node : m.nodes) {
        auto lr(node.lodRange());
        std::cout << "range<" << node.srs << ">: " << lr << " "
                  << lr.max << "/" << node.tileRange(lr.max)
                  << '\n';
    }

    std::cout << "range: " << m.lodRange << " " << m.tileRange << '\n';
    std::cout << std::fixed << "position: " << m.position << '\n';
    std::cout << std::flush;

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    return Calipers()(argc, argv);
}
