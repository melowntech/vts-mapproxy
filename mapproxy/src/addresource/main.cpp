/**
 * Copyright (c) 2018 Melown Technologies SE
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
#include <boost/regex.hpp>

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>

#include "utility/streams.hpp"
#include "utility/tcpendpoint-io.hpp"
#include "utility/buildsys.hpp"
#include "utility/openmp.hpp"
#include "utility/progress.hpp"
#include "utility/path.hpp"
#include "utility/enum-io.hpp"
#include "service/cmdline.hpp"

#include "geo/geodataset.hpp"
#include "gdal-drivers/register.hpp"

#include "vts-libs/registry/po.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/tileindex.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace vts = vtslibs::vts;

namespace vr = vtslibs::registry;

UTILITY_GENERATE_ENUM_CI(DatasetType,
                         ((dem)("dem")("dsm"))
                         ((ophoto))
)

class AddResource : public service::Cmdline {
public:
    AddResource()
        : service::Cmdline("addresource", BUILD_TARGET_VERSION)
    {}

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    std::string referenceFrame_;

    fs::path mapproxyDataRoot_;
    fs::path mapproxyDefinitionDir_;
    fs::path mapproxyCtrl_;

    fs::path dataset_;

    boost::optional<DatasetType> datasetType_;
};

void AddResource::configuration(po::options_description &cmdline
                                , po::options_description &config
                                , po::positional_options_description &pd)
{
    vr::registryConfiguration(config, vr::defaultPath());

    cmdline.add_options()
        ("referenceFrame", po::value(&referenceFrame_)->required()
         , "Reference frame.")
        ("dataset", po::value(&dataset_)->required()
         , "Path to input raster dataset.")
        ("datasetType", po::value<DatasetType>()
         , "Dataset type (dem or ophoto). Mandatory only "
         "if autodetect fails.")
        ;

    config.add_options()
        ("mapproxy.dataRoot", po::value(&mapproxyDataRoot_)->required()
         , "Path to mapproxy resource data root directory.")
        ("mapproxy.definitionDir"
         , po::value(&mapproxyDefinitionDir_)->required()
         , "Path to mapproxy resource definition directory.")
        ("mapproxy.ctrl", po::value(&mapproxyCtrl_)->required()
         , "Path to mapproxy control socket.")
        ;

#if 0
        ("dataset", po::value(&dataset_)->required()
         , "Path to ORG dataset to proces.")
        ("output", po::value(&output_)->required()
         , "Path output mask file.")

        ("lod", po::value(&lod_)->required()
         , "Bottom level of detail.")
        ("dilate"
         , po::value(&dilate_)->required()->default_value(dilate_)
         , "Dilation distance (in SDS SRS units).")
        ("segments"
         , po::value(&segments_)->required()->default_value(segments_)
         , "The number of segments used to approximate a 90 degree "
         "(quadrant) of curvature.")

        ("tileSizeOrder"
         , po::value(&tileSizeOrder_)->required()
         ->default_value(tileSizeOrder_)
         , "Raster tile size as a binary order. Tile size is "
         "(1 << tileSizeOrder, 1 << tileSizeOrder).")

        ("workBlock"
         , po::value(&workBlock_)->required()->default_value(workBlock_)
         , "Block of tiles to process at once. Specified in binary exponent. "
         "Used blocks size is (1 << workBlock, 1 << workBlock)."
         )

        ("generateGdalDatasets"
         , po::value(&generateGdalDatasets_)->required()
         ->default_value(false)->implicit_value(true)
         , "Generates gdal dataset per reference frame node."
         )

        ("layer"
         , po::value(&layer_)->required()->default_value(layer_)
         , "Layer selector. Not needed if there is just one layer "
         "in the input."
         )

        ("featureFilter"
         , po::value<std::string>()
         , "Feature fileter (format: FIELD=REGEX): rasterize only geometries "
         "from features that contain field named FIELD which value matches "
         "given REGEX. All features are rasterized by default.")
        ;

#endif
    pd.add("dataset", 1)
        ;

    (void) pd;
}

void AddResource::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    if (vars.count("datasetType")) {
        datasetType_ = vars["datasetType"].as<DatasetType>();
    }

    LOG(info3, log_)
        << "Config:"
        << "\nmapproxy.dataRoot = " << mapproxyDataRoot_
        << "\nmapproxy.definitionDir = " << mapproxyDefinitionDir_
        << "\nmapproxy.ctrl = " << mapproxyCtrl_
        << "\nreferenceFrame = " << referenceFrame_
        << "\n"
        ;
}

bool AddResource::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("mapproxy addresource tool\n"
                "    Preprocesses and adds new mapproxy resource to "
                "extisting mapproxy installation.\n"
                "\n"
                );

        return true;
    }

    return false;
}

DatasetType detectType(const geo::GeoDataset::Descriptor &ds
                       , const boost::optional<DatasetType> &forcedType)
{
    if (forcedType) { return *forcedType; }

    if (ds.bands >= 3) { return DatasetType::ophoto; }

    if (ds.bands != 1) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot autodetect dataset type, unsupported number of bands ("
            << ds.bands << ").";
    }

    if (ds.dataType == GDT_Byte) {
        // probably monochromatic orthophoto
        return DatasetType::ophoto;
    }

    // anything else is DEM
    return DatasetType::dem;
}

int AddResource::run()
{
    // find reference frame
    auto rf(vr::system.referenceFrames(referenceFrame_));

    const auto ds(geo::GeoDataset::open(dataset_).descriptor());
    const auto datasetType(detectType(ds, datasetType_));

    LOG(info3) << "Treating input dataset " << dataset_
               << " as " << datasetType << ".";

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    return AddResource()(argc, argv);
}
