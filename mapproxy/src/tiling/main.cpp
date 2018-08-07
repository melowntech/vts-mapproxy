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
#include <boost/variant.hpp>

#include "utility/streams.hpp"
#include "utility/buildsys.hpp"
#include "utility/openmp.hpp"
#include "service/cmdline.hpp"

#include "geo/geodataset.hpp"
#include "geo/gdal.hpp"

#include "gdal-drivers/register.hpp"

#include "vts-libs/registry/po.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/tileindex.hpp"

#include "./tiling.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace vts = vtslibs::vts;

namespace vr = vtslibs::registry;

struct UnifiedTileRange {
    boost::variant<vts::TileRange, vts::LodTileRange> range;

    UnifiedTileRange(vts::TileRange &&tr)
        : range(std::move(tr))
    {}

    UnifiedTileRange(vts::LodTileRange &&tr)
        : range(std::move(tr))
    {}

    typedef std::vector<UnifiedTileRange> list;
};

class PrintUnifiedTileRange : public boost::static_visitor<>
{
public:
    PrintUnifiedTileRange(std::ostream &os) : os_(&os) {}

    void operator()(const vts::TileRange &tr) const {
        *os_ << tr;
    }

    void operator()(const vts::LodTileRange &tr) const {
        *os_ << tr;
    }

private:
    std::ostream *os_;
};


template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const UnifiedTileRange &tr)
{
    boost::apply_visitor(PrintUnifiedTileRange(os), tr.range);
    return os;
}

class AsLodTileRange : public boost::static_visitor<vts::LodTileRange>
{
public:
    AsLodTileRange(vts::Lod minLod) : minLod_(minLod) {}

    vts::LodTileRange operator()(const vts::TileRange &tr) const {
        return vts::LodTileRange(minLod_, tr);
    }

    vts::LodTileRange operator()(const vts::LodTileRange &tr) const {
        return tr;
    }

private:
    vts::Lod minLod_;
};

vts::LodTileRange::list
asLodTileRangeList(vts::Lod minLod, const UnifiedTileRange::list &utr)
{
    vts::LodTileRange::list ranges;
    AsLodTileRange visitor(minLod);
    for (const auto &tr : utr) {
        ranges.push_back(boost::apply_visitor(visitor, tr.range));
    }
    return ranges;
}

void validate(boost::any &v, const std::vector<std::string> &values
              , UnifiedTileRange*, int)
{
    po::validators::check_first_occurrence(v);
    const auto &s(po::validators::get_single_string(values));

    try {
        v = UnifiedTileRange(boost::lexical_cast<vts::TileRange>(s));
    } catch (const boost::bad_lexical_cast&) {
        try {
            v = UnifiedTileRange(boost::lexical_cast<vts::LodTileRange>(s));
        } catch (const boost::bad_lexical_cast&) {
            throw po::validation_error
                (po::validation_error::invalid_option_value);
        }
    }
}

class Tiling : public service::Cmdline {
public:
    Tiling()
        : service::Cmdline("mapproxy-tiling", BUILD_TARGET_VERSION
                           , (service::DISABLE_EXCESSIVE_LOGGING))
        , noexcept_(false)
    {}

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    int runImpl();

    fs::path input_;
    fs::path output_;
    std::string referenceFrame_;
    vts::LodRange lodRange_;
    vts::TileRange tileRange_;

    UnifiedTileRange::list tileRanges_;

    tiling::Config config_;
    fs::path dataset_;

    bool noexcept_;
};

void Tiling::configuration(po::options_description &cmdline
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
        ("referenceFrame", po::value(&referenceFrame_)->required()
         , "Tiling reference frame.")
        ("lodRange", po::value(&lodRange_)->required()
         , "Lod range where content is generated.")
        ("tileRange", po::value(&tileRanges_)->required()
         , "Either single tile range at lodRange.min or one or more "
         "lod/tileRange entries (obtained from mapproxy-calipers).")
        ("tileSampling", po::value(&config_.tileSampling)
         ->default_value(config_.tileSampling)
         , "Nuber of pixels to break tile into when analyzing its coverage.")
        ("parallel", po::value(&config_.parallel)
         ->default_value(config_.parallel)
         , "Use OpenMP to parallelize work.")
        ("forceWatertight", po::value(&config_.forceWatertight)
         ->default_value(config_.forceWatertight)->implicit_value(true)
         , "Treats all partial tiles as watertight. Will lie about the holes "
           "in the dataset.")

        ("noexcept", "Do not catch exceptions, let the program crash.")
        ;

    pd.add("input", 1)
        .add("referenceFrame", 1)
        ;

    (void) config;
}

void Tiling::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    bool complexDataset(true);

    if (fs::exists(input_ / "dem")) {
        dataset_ = input_ / "dem";
    } else if (fs::exists(input_ / "ophoto")) {
        dataset_ = input_ / "ophoto";
    } else {
        dataset_ = input_;
        complexDataset = false;
    }

    if (vars.count("output")) {
        output_ = vars["output"].as<fs::path>();
    } else if (complexDataset) {
        output_ = input_ / ("tiling." + referenceFrame_);
    } else {
        throw po::required_option("output");
    }

    noexcept_ = vars.count("noexcept");

    LOG(info3, log_)
        << "Config:"
        << "\n\tinput = " << input_
        << "\n\tdataset = " << dataset_
        << "\n\toutput = " << output_
        << "\n\treferenceFrame = " << referenceFrame_
        << "\n\tlodRange = " << lodRange_
        << "\n\ttileRange = " << utility::join(tileRanges_, " ")
        << "\n"
        ;
}

bool Tiling::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("mapproxy-tiling tool\n"
                "    Analyzes input dataset and generates tiling "
                "information.\n"
                "usage:\n"
                "    mapproxy-tiling input referenceFrame [ options ]\n"
                "\n"
                "    LOD and tile ranges:\n"
                "        Tile tree descent must be limited by user.\n"
                "        LOD range (--lodRange) is a range of levels of\n"
                "        detail where tiling tree is analyzed in the\n"
                "        \"min,max\" format; both numbers are inclusive.\n"
                "        Tile range (--tileRange) is a range of tiles at\n"
                "        certain LOD in the \"xmin,ymin:xmax,ymax\" format;\n"
                "        all four numbers are inclusive.\n"
                "        Simple tile range describes tile range at minimum\n"
                "        LOD range (i.e. first number in LOD range).\n"
                "        Complex tile range is specified in LOD/tileRange\n"
                "        format which describes tile range at given LOD \n"
                "        (which can even be outside of lodRange).\n"
                "\n"
                "        It is recommended to use LOD range and complex tile\n"
                "        ranges from the \"mapproxy-calipers\" tool output\n"
                "        instead of guessing values or processing whole tile \n"
                "        world.\n"
                "\n"
                "        NB: For unlimited tile tree descent (i.e. whole world)\n"
                "        one must explicitely use lodRange starting from zero \n"
                "        and use --tileRange=0,0:0,0.\n"
                "\n"
                );

        return true;
    }

    return false;
}

int Tiling::runImpl()
{
    auto rf(vr::system.referenceFrames(referenceFrame_));

    auto ds(geo::GeoDataset::open(dataset_));

    auto ti(tiling::generate(dataset_, rf, lodRange_
                             , asLodTileRangeList(lodRange_.min, tileRanges_)
                             , config_));

    LOG(info3) << "Saving generated tile index into " << output_ << ".";
    ti.save(output_);
    LOG(info3) << "Tile index saved.";

    return EXIT_SUCCESS;
}

int Tiling::run()
{
    if (noexcept_) {
        return runImpl();
    }

    try {
        return runImpl();
    } catch (const std::exception &e) {
        std::cerr << "maproxy-tiling: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    // force VRT not to share undelying datasets
    geo::Gdal::setOption("VRT_SHARED_SOURCE", 0);
    return Tiling()(argc, argv);
}
