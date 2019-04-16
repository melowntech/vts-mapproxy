/**
 * Copyright (c) 2019 Melown Technologies SE
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
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/trim.hpp>

#include "utility/streams.hpp"
#include "utility/buildsys.hpp"
#include "utility/path.hpp"
#include "utility/enum-io.hpp"
#include "utility/path.hpp"
#include "utility/filesystem.hpp"
#include "utility/format.hpp"
#include "utility/md5.hpp"

#include "service/cmdline.hpp"
#include "service/ctrlclient.hpp"
#include "service/pidfile.hpp"

#include "gdal-drivers/register.hpp"

#include "vts-libs/registry/po.hpp"

// mapproxy stuff
#include "mapproxy/support/tileindex.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace vts = vtslibs::vts;
namespace vr = vtslibs::registry;


class GenerateTileIndex : public service::Cmdline {
public:
    GenerateTileIndex()
        : service::Cmdline("generate-tileindex", BUILD_TARGET_VERSION)
        , navtiles_(false), resource_({})
    {}

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    fs::path tiling_;
    bool navtiles_;
    Resource resource_;
    fs::path output_;
};

void GenerateTileIndex::configuration(po::options_description &cmdline
                                      , po::options_description &config
                                      , po::positional_options_description &pd)
{
    vr::registryConfiguration(config, vr::defaultPath());

    cmdline.add_options()
        ("tiling", po::value(&tiling_)->required()
         , "Path to reference frame tiling.")
        ("referenceFrame", po::value(&resource_.id.referenceFrame)->required()
         , "Reference frame.")
        ("resouce.group", po::value(&resource_.id.group)->default_value("id")
         , "Resource identifier.")
        ("resouce.id", po::value(&resource_.id.id)->default_value("group")
         , "Resource group.")

        ("resouce.id", po::value(&resource_.id.id)->required()
         , "Resource group.")
        ("lodRange", po::value(&resource_.lodRange)->required()
         , "Valid LOD range.")
        ("tileRange", po::value(&resource_.tileRange)->required()
         , "Valid tile range at lodRange.min.")
        ("output", po::value(&output_)->required()
         , "Output file.")
        ;

    (void) config;
    (void) pd;
}

void GenerateTileIndex::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    resource_.referenceFrame
        = &vr::system.referenceFrames(resource_.id.referenceFrame);
}

bool GenerateTileIndex::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("Generate tileindex from lod/tile ranges and reference frame "
                "tiling.\n"
                );

        return true;
    }

    return false;
}

int GenerateTileIndex::run()
{

    vts::TileIndex out;
    prepareTileIndex(out, &tiling_, resource_, navtiles_);
    out.save(output_);

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    // force VRT not to share underlying datasets
    gdal_drivers::registerAll();
    return GenerateTileIndex()(argc, argv);
}
