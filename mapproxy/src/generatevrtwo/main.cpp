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
#include <numeric>

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include <gdal/vrtdataset.h>

#include "cpl_minixml.h"

#include "utility/streams.hpp"
#include "utility/buildsys.hpp"
#include "utility/openmp.hpp"
#include "utility/raise.hpp"
#include "utility/duration.hpp"
#include "utility/time.hpp"
#include "service/cmdline.hpp"
#include "utility/enum-io.hpp"
#include "utility/path.hpp"

#include "geo/geodataset.hpp"
#include "geo/gdal.hpp"
#include "gdal-drivers/register.hpp"

#include "./generatevrtwo.hpp"
#include "./io.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace def {

std::vector<std::string> createOptions {
    "COMPRESS=DEFLATE"
    , "PREDICTOR" // if not set, predictor will be set to 2/3 for int/float
 // buggy in GDAL 2.1, some tiles are not writen
 // , "NUM_THREADS=ALL_CPUS"
    , "ZLEVEL=9"
};

const math::Size2 tileSize(4096, 4096);

const math::Size2 minOvrSize(2, 2);

} // namespace def

class VrtWo : public service::Cmdline {
public:
    VrtWo()
        : service::Cmdline("generageVrtWo", BUILD_TARGET_VERSION)
    {
    }

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    fs::path input_;
    fs::path output_;
    std::vector<std::string> co_;
    vrtwo::Config config_;
};

void VrtWo::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Path to input GDAL dataset.")
        ("output", po::value(&output_)->required()
         , "Path to output directory where to place "
         "GDAL dataset and its overviews.")
        ("tileSize", po::value(&config_.tileSize)
         ->default_value(config_.tileSize)->required()
        , "Tile size (size of individual GTiff files).")
        ("resampling", po::value(&config_.resampling)->required()
         , utility::concat
         ("Resampling algorithm. One of ["
          , enumerationString(decltype(config_.resampling)())
          , "].").c_str())
        ("minOvrSize", po::value(&config_.minOvrSize)->required()
         ->default_value(config_.minOvrSize)
        , "Minimum size of generated overview. Overview generation is stoppend"
         " when either width of height of last overview reaches given limit.")
        ("overwrite", po::value(&config_.overwrite)->required()
         ->default_value(false)->implicit_value(true)
        , "Overwrite existing dataset.")
        ("wrapx", po::value<int>()
         ->implicit_value(0)
        , "Wrap dataset in X direction. Optional. Value indicates number "
         "of overlapping pixels. Useful only for global datasets spanning "
         "longitude from -180 to +180. Probably could be guessed from input "
         "data in future releases.")
        ("background", po::value<vrtwo::Color>()
        , "Optional background. If whole warped tile contains this "
         "color it is left empty in the output. Solid dataset with this color "
         "is created and places as a first source for each band in "
         "all overviews.")
        ("co", po::value(&co_)
        , utility::concat
         ("GTiff extra create option; can be used multiple times. "
          "If no extra option is specified a compiled-in default is "
          "used instead (", utility::join(def::createOptions, ", "), ").")
         .c_str())
        ("nodata", po::value<std::string>()
         , "Optional nodata value override. Can be NONE (to disable any "
         "nodata value) or a (real) number. Original input dataset's nodata "
         "value is used if not specified.")
        ;

    pd.add("input", 1)
        .add("output", 1)
        ;

    (void) config;
}

void VrtWo::configure(const po::variables_map &vars)
{
    if (vars.count("background")) {
        config_.background = vars["background"].as<vrtwo::Color>();
    }

    if (vars.count("wrapx")) {
        config_.wrapx = vars["wrapx"].as<int>();
    }

    if (!vars.count("co")) {
        co_ = def::createOptions;
    }

    input_ = fs::absolute(input_);

    // prepare create options
    config_.createOptions("TILED", true);
    for (const auto &option : co_) {
        auto split(option.find('='));
        if (split == std::string::npos) {
            config_.createOptions(option, "");
        } else {
            config_.createOptions(option.substr(0, split)
                                  , option.substr(split + 1));
        }
    }

    if (vars.count("nodata")) {
        // get raw value
        auto raw(vars["nodata"].as<std::string>());

        try {
            // interpret as a double
            config_.nodata
                = geo::NodataValue(boost::lexical_cast<double>(raw));
        } catch (const boost::bad_lexical_cast&) {
            // not a double, check special case
            // convert to lowercase
            ba::to_lower(raw);
            if (raw == "none") {
                config_.nodata = geo::NodataValue();
            }
        }
    }

    // sanitize min ovr size
    if (config_.minOvrSize.width <= 1) { config_.minOvrSize.width = 2; }
    if (config_.minOvrSize.height <= 1) { config_.minOvrSize.height = 2; }

    LOG(info3, log_)
        << "Config:"
        << "\n\tinput = " << input_
        << "\n\toutput = " << output_
        << "\n\ttileSize = " << config_.tileSize
        << "\n\tresampling = " << config_.resampling
        << "\n\tminOvrSize = " << config_.minOvrSize
        << "\n\twrapx = "
        << utility::LManip([&](std::ostream &os) -> std::ostream& {
                if (config_.wrapx) { return os << "true, " << *config_.wrapx; }
                return os << "false";
            })
        << "\n\tbackground = " << config_.background
        << "\n\tco = " << utility::join(co_, ", ")
        << utility::LManip([&](std::ostream &os) -> std::ostream& {
                if (!config_.nodata) { return os; }

                os << "\n\tnodata = ";
                if (*config_.nodata) {
                    os << **config_.nodata;
                } else {
                    os << "none";
                }
                return os;
            })
        << "\n"
        ;
}

bool VrtWo::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("generatevrtwo input output [options]\n"
                "    Generates virtual GDAL dataset with overviews.\n"
                "\n"
                );

        return true;
    }

    return false;
}

int VrtWo::run()
{
    vrtwo::generate(input_, output_, config_);

    LOG(info4) << "VRT with overviews in " << output_
               << " successfully generated.";
    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    // force VRT not to share undelying datasets
    geo::Gdal::setOption("VRT_SHARED_SOURCE", 0);
    geo::Gdal::setOption("GDAL_TIFF_INTERNAL_MASK", "YES");
    return VrtWo()(argc, argv);
}
