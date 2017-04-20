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

#include "utility/streams.hpp"
#include "utility/buildsys.hpp"
#include "service/cmdline.hpp"

#include "vts-libs/registry/po.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/tileindex.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace vts = vtslibs::vts;

namespace vr = vtslibs::registry;

class DemTiling : public service::Cmdline {
public:
    DemTiling()
        : service::Cmdline("sub-tiling", BUILD_TARGET_VERSION)
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

    vts::TileId root_;
};

void DemTiling::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    vr::registryConfiguration(cmdline, vr::defaultPath());

    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Path to dataset to proces.")
        ("output", po::value<fs::path>(&output_)
         , "Path output tiling file if different from "
         "input/tiling.referenceFrame.")
        ("root", po::value(&root_)->required()
         , "Root of dem tiling subtree.")
        ;

    pd.add("input", 1)
        .add("output", 1)
        ;

    (void) config;
}

void DemTiling::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    LOG(info3, log_)
        << "Config:"
        << "\n\tinput = " << input_
        << "\n\toutput = " << output_
        << "\n"
        ;
}

bool DemTiling::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("mapproxy-sub-tiling tool\n"
                "    Cuts dem tiling subtree.\n"
                "usage:\n"
                "    mapproxy-sub-tiling input output --root rootId "
                "[ options ]\n"
                "\n"
                );

        return true;
    }

    return false;
}


int DemTiling::run()
{
    vts::TileIndex tiTmp;
    tiTmp.load(input_);

    const vts::TileIndex &ti(tiTmp);

    // output
    vts::TileIndex to;

    auto lr(ti.lodRange());

    lr.min = root_.lod;
    if (lr.min > lr.max) {
        LOG(warn3) << "Root " << root_ << " is below valid data, "
                   << "saving empty tile index.";
        to.save(output_);
        return EXIT_SUCCESS;
    }

    // for each lod in new tile range
    vts::Lod outLod(0);
    vts::TileId reference(root_);
    for (auto lod : lr) {
        ti.tree(lod)->forEachNode([&](int x, int y, int size
                                      , vts::QTree::value_type value)
        {
            // shift
            x -= reference.x;
            y -= reference.y;

            // set block
            to.set(outLod, vts::TileRange(x, y, x + size, y + size)
                   , value);
        }, vts::QTree::Filter::white);

        // next lod
        ++outLod;
        reference = vts::lowestChild(reference);
    }

    LOG(info3) << "Saving generated tile index into " << output_ << ".";
    to.save(output_);
    LOG(info3) << "Tile index saved.";

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    return DemTiling()(argc, argv);
}
