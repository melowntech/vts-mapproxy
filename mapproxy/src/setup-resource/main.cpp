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

#include <unicode/translit.h>
#include <unicode/unistr.h>
#include <unicode/ucnv.h>

#include "utility/streams.hpp"
#include "utility/tcpendpoint-io.hpp"
#include "utility/buildsys.hpp"
#include "utility/openmp.hpp"
#include "utility/progress.hpp"
#include "utility/path.hpp"
#include "utility/enum-io.hpp"
#include "utility/path.hpp"

#include "service/cmdline.hpp"

#include "geo/geodataset.hpp"
#include "gdal-drivers/register.hpp"

#include "vts-libs/registry/po.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/tileindex.hpp"

#include "calipers/calipers.hpp"
#include "mapproxy/resource.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace vts = vtslibs::vts;

namespace vr = vtslibs::registry;

UTILITY_GENERATE_ENUM_CI(ResourceType,
                         ((tms)("TMS"))
                         ((tin)("TIN"))
                         )

class SetupResource : public service::Cmdline {
public:
    SetupResource()
        : service::Cmdline("mapproxy-setup-resource", BUILD_TARGET_VERSION)
    {}

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    fs::path mapproxyDataRoot_;
    fs::path mapproxyDefinitionDir_;
    fs::path mapproxyCtrl_;

    std::string dataset_; // do not use fs::path since it needs quotes in case
                          // of spaces in filename

    ResourceType resourceType_;

    Resource::Id resourceId_;
};

void SetupResource::configuration(po::options_description &cmdline
                                , po::options_description &config
                                , po::positional_options_description &pd)
{
    vr::registryConfiguration(config, vr::defaultPath());

    cmdline.add_options()
        ("referenceFrame", po::value(&resourceId_.referenceFrame)->required()
         , "Reference frame.")
        ("id", po::value(&resourceId_.id)
         , "Resource ID. Deduced from filename if not provided.")
        ("group", po::value(&resourceId_.group)
         , "Resource group ID. Deduced from filename if not provided.")
        ("dataset", po::value(&dataset_)->required()
         , "Path to input raster dataset.")
        ("resourceType", po::value<ResourceType>()
         , "Resource type: TMS or TIN.")
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

    pd.add("dataset", 1)
        ;

    (void) pd;
}

void SetupResource::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    if (vars.count("resourceType")) {
        resourceType_ = vars["resourceType"].as<ResourceType>();
    }

    LOG(info3, log_)
        << "Config:"
        << "\nmapproxy.dataRoot = " << mapproxyDataRoot_
        << "\nmapproxy.definitionDir = " << mapproxyDefinitionDir_
        << "\nmapproxy.ctrl = " << mapproxyCtrl_
        << "\nreferenceFrame = " << resourceId_.referenceFrame
        << "\n"
        ;
}

bool SetupResource::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("mapproxy resource setup tool\n"
                "    Sets up and adds new mapproxy resource to "
                "extisting mapproxy installation.\n"
                "\n"
                );

        return true;
    }

    return false;
}

boost::optional<calipers::DatasetType>
asDatasetType(const boost::optional<ResourceType> &type)
{
    if (!type) { return boost::none; }
    switch (*type) {
    case ResourceType::tms: return calipers::DatasetType::ophoto;
    case ResourceType::tin: return calipers::DatasetType::dem;
    }
    return boost::none;
}

Resource::Id deduceResourceId(fs::path path, Resource::Id resourceId)
{
    // pass if already valid
    if (!(resourceId.id.empty() || resourceId.group.empty())) {
        return resourceId;
    }

    path = fs::absolute(path);
    const auto dir(utility::sanitizePath(path.parent_path().filename())
                   .string());
    const auto stem(utility::sanitizePath(path.stem()).string());

    if (resourceId.group.empty()) { resourceId.group = dir; }
    if (resourceId.id.empty()) { resourceId.id = stem; }

    return resourceId;
}

int SetupResource::run()
{
    // find reference frame
    const auto *rf(vr::system.referenceFrames
                   (resourceId_.referenceFrame, std::nothrow));
    if (!rf) {
        LOG(fatal)
            << "There is no reference frame with ID <"
            << resourceId_.referenceFrame << ">.";
        return EXIT_FAILURE;
    }

    // open dataset
    const auto ds(geo::GeoDataset::open(dataset_));

    // 1) deduce resource ID
    const auto resourceId(deduceResourceId(dataset_, resourceId_));
    LOG(info4) << "Using resource ID <" << resourceId << ">.";

    // 2) check resource existence in mapproxy

    // 3) measure dataset
    calipers::Config calipersConfig;
    calipersConfig.datasetType = asDatasetType(resourceType_);
    const auto m(calipers::measure(*rf, ds.descriptor(), calipersConfig));

    if (m.nodes.empty()) {
        LOG(fatal)
            << "Unable to set up a mapproxy resource <" << resourceId
            << "> from " << dataset_ << ".";
        return EXIT_FAILURE;
    }

    LOG(info4) << m.tileRange;

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    return SetupResource()(argc, argv);
}
