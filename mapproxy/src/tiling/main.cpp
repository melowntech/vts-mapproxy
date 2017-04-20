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
#include "utility/tcpendpoint-io.hpp"
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

struct Flag {
    typedef vts::TileIndex::Flag::value_type value_type;
    enum : value_type {
        descend = 0x1
        , analyze = 0x2
    };
};

class Tiling : public service::Cmdline {
public:
    Tiling()
        : service::Cmdline("mapproxy-tiling", BUILD_TARGET_VERSION
                           , (service::DISABLE_EXCESSIVE_LOGGING))
        , tileSampling_(128), parallel_(true), forceWatertight_(false)
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
    int tileSampling_;

    UnifiedTileRange::list tileRanges_;

    fs::path dataset_;
    bool parallel_;
    bool forceWatertight_;

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
        // ("tileRange", po::value(&tileRange_)->required()
        //  , "Tile range at min lod where content is generated.")
        ("tileRange", po::value(&tileRanges_)->required()
         , "Tile range at min lod where content is generated.")
        ("tileSampling", po::value(&tileSampling_)
         ->default_value(tileSampling_)
         , "Nuber of pixels to break tile into when analyzing its coverage.")
        ("parallel", po::value(&parallel_)
         ->default_value(parallel_)
         , "Use OpenMP to parallelize work.")
        ("forceWatertight", po::value(&forceWatertight_)
         ->default_value(forceWatertight_)->implicit_value(true)
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

    if (exists(input_ / "dem")) {
        dataset_ = input_ / "dem";
    } else if (exists(input_ / "ophoto")) {
        dataset_ = input_ / "ophoto";
    }

    if (vars.count("output")) {
        output_ = vars["output"].as<fs::path>();
    } else {
        output_ = input_ / ("tiling." + referenceFrame_);
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
                );

        return true;
    }

    return false;
}

math::Extents2 extentsPlusHalfPixel(const math::Extents2 &extents
                                    , int pixels)
{
    auto es(math::size(extents));
    const math::Size2f px(es.width / pixels, es.height / pixels);
    const math::Point2 hpx(px.width / 2, px.height / 2);
    return math::Extents2(extents.ll - hpx, extents.ur + hpx);
}

typedef vts::TileIndex::Flag TiFlag;

class TreeWalker {
public:
    TreeWalker(vts::TileIndex &ti, const fs::path &dataset
               , const vts::NodeInfo &root
               , const vts::LodRange &lodRange
               , const vts::LodTileRange::list &tileRanges
               , int tileSampling, bool parallel, bool forceWatertight);

private:
    void process(bool parentProductive
                 , const vts::NodeInfo &node, double upscaling = 0.0);
    void descend(const vts::NodeInfo &node, const vts::TileId &tileId
                 , double upscaling);

    const geo::GeoDataset& dataset() {
        return gds_[omp_get_thread_num()];
    }

    void prepareDataset() {
        int count(omp_get_num_threads());
        gds_.reserve(count);
        for (int i(0); i < count; ++i) {
            gds_.emplace_back(geo::GeoDataset::open(dataset_));
        }
    }

    void buildWorld(const vts::LodRange &lodRange
                    , const vts::LodTileRange::list &tileRanges);

    const fs::path dataset_;
    vts::TileIndex &ti_;
    vts::LodRange lodRange_;
    int tileSampling_;

    std::vector<geo::GeoDataset> gds_;
    bool parallel_;
    bool forceWatertight_;

    vts::TileIndex world_;
};


TreeWalker::TreeWalker(vts::TileIndex &ti, const fs::path &dataset
                       , const vts::NodeInfo &root
                       , const vts::LodRange &lodRange
                       , const vts::LodTileRange::list &tileRanges
                       , int tileSampling, bool parallel, bool forceWatertight)
    : dataset_(dataset), ti_(ti), lodRange_(lodRange)
    , tileSampling_(tileSampling)
    , parallel_(parallel), forceWatertight_(forceWatertight)
{
    buildWorld(lodRange, tileRanges);

    if (parallel_) {
        UTILITY_OMP(parallel)
            UTILITY_OMP(single)
            {
                prepareDataset();
                process(false, root);
            }
    } else {
        prepareDataset();
        process(false, root);
    }
}

void TreeWalker::buildWorld(const vts::LodRange &lodRange
                            , const vts::LodTileRange::list &tileRanges)
{
    // process whole LOD range from root
    for (auto lod : vts::LodRange(0, lodRange.max)) {
        // set mesh flag everywhere we have to step in
        // set to watertight mesh everywhere we should analyze data
        Flag::value_type flags(Flag::descend);
        if (lod >= lodRange.min) {
            flags |= Flag::analyze;
        }

        for (const auto &tr : tileRanges) {
            auto localRange(vts::shiftRange(tr, lod));
            world_.set(lod, localRange, flags);
        }
    }
}

void TreeWalker::descend(const vts::NodeInfo &node
                         , const vts::TileId &tileId, double upscaling)
{
    if (tileId.lod == lodRange_.max) {
        // no children down there
        return;
    }

    // do not use const otherwise OpenMP makes it shared
    bool parentProductive(node.productive());

    // we can proces children -> go down
    for (auto child : vts::children(tileId)) {
        // compute child node
        auto childNode(node.child(child));

        if (parallel_) {
            UTILITY_OMP(task)
                process(parentProductive, childNode, upscaling);
        } else {
            process(parentProductive, childNode, upscaling);
        }
    }
}

void TreeWalker::process(bool parentProductive
                         , const vts::NodeInfo &node, double upscaling)
{
    struct TIDGuard {
        TIDGuard(const std::string &id)
            : old(dbglog::thread_id())
        {
            dbglog::thread_id(id);
        }
        ~TIDGuard() { dbglog::thread_id(old); }

        const std::string old;
    };

    const auto tileId(node.nodeId());
    if ((tileId.lod > lodRange_.max)) {
        // outside of configured area
        return;
    }

    auto fullSubtree([&]()
    {
        UTILITY_OMP(critical(tileIndex))
        {
            ti_.set(vts::LodRange(tileId.lod, lodRange_.max)
                    , vts::tileRange(tileId)
                    , (TiFlag::mesh | TiFlag::watertight));
        }

        return;
    });

    TIDGuard tg(str(boost::format("tile:%s") % tileId));

    if (!node.valid()) {
        // Node is invalid but we can safely say that we have watertight mesh
        // here and down to the maximum LOD to save space in tile index.
        //
        // It is a lie but it will not hurt anyone.

        // set full subtree
        if ((tileId.lod >= lodRange_.min) && parentProductive) {
            (void) parentProductive;
            fullSubtree();
            // stop descent here
            LOG(info3)
                << "Processed tile " << tileId
                << " (extents: " << std::fixed << node.extents()
                << ", srs: " << node.srs()
                << ") [fake watertight subtree in invalid part of a tree].";
        }
        return;
    }

    if (!node.productive()) {
        // unproductive node, immediate descend
        descend(node, tileId, upscaling);
        return;
    }

    LOG(info2) << "Processing tile " << tileId << ".";

    int samples(tileSampling_);

    // upscaling, each level decreases size by half
    // use at least 8 samples
    if (upscaling >= 1.0) {
        samples = std::max(int(samples / upscaling), 8);
    }

    math::Size2 size(samples + 1, samples + 1);

    // consult flags
    const auto flags(world_.get(tileId));
    if (!flags) {
        // outside of defined world
        LOG(info1) << "outside of defined world";
        return;
    }

    if (flags & Flag::analyze) {
        // warp input dataset into tile
        const auto &ds(dataset());
        // set some output nodata value to force mask generation
        auto tileDs(geo::GeoDataset::deriveInMemory
                    (ds, node.srsDef(), size
                     , extentsPlusHalfPixel(node.extents(), samples)
                     , GDT_Float32
                     , geo::GeoDataset::NodataValue(-1e6)));

        geo::GeoDataset::WarpOptions wo;
        if (upscaling >= 1.0) {
            // last level used original dataset, we have to enforce it now as
            // well otherwise GeoDataset may choose some overlay automatically
            // based on smaller size
            wo.overview = geo::Overview();
        }

        auto wri([&]() -> geo::GeoDataset::WarpResultInfo
        {

            try {
                // try to warp as in mapproxy
                return ds.warpInto
                    (tileDs, geo::GeoDataset::Resampling::dem, wo);
            } catch (const geo::WarpError &e) {
                // failed -> average could help
                LOG(info3)
                    << "Warp failed (" << e.what()
                    << "), restarting with simpler kernel.";
                return ds.warpInto
                    (tileDs, geo::GeoDataset::Resampling::average, wo);
            }
        }());

        TiFlag::value_type baseFlags(TiFlag::mesh);
        if (!upscaling) {
            baseFlags |= TiFlag::navtile;
        }

        // grab mask of warped dataset
        const auto &mask(tileDs.cmask());

        auto checkMask([&]() -> vts::NodeInfo::CoveredArea {
            auto res(node.checkMask(mask, vts::NodeInfo::CoverageType::grid, 1));

            if (forceWatertight_ && (res == vts::NodeInfo::CoveredArea::some)) {
                return vts::NodeInfo::CoveredArea::whole;
            }

            return res;
        });

        switch (checkMask()) {
        case vts::NodeInfo::CoveredArea::whole: {
            // fully covered by dataset and by reference frame definition

            if (!wri.overview && wri.truescale >= 1.0) {
                // warped using original dataset and no downscaling
                // which could possibly fill holes, no holes -> always without
                // holes -> set whole subtree to watertight mesh

                fullSubtree();

                // stop descent here
                LOG(info3)
                    << "Processed tile " << tileId
                    << " (extents: " << std::fixed << node.extents()
                    << ", srs: " << node.srs()
                    << ") [watertight subtree].";
                return;
            }

            UTILITY_OMP(critical(tileIndex))
                ti_.set(tileId, (baseFlags | TiFlag::watertight));
            LOG(info3)
                << "Processed tile " << tileId
                << " (extents: " << std::fixed << node.extents()
                << ", srs: " << node.srs()
                << ") [watertight].";
            break;
        }

        case vts::NodeInfo::CoveredArea::some: {
            // partially covered
            UTILITY_OMP(critical(tileIndex))
                ti_.set(tileId, baseFlags);
            LOG(info3)
                << "Processed tile " << tileId
                << " (extents: " << std::fixed << node.extents()
                << ", srs: " << node.srs()
                << ") [partial].";
            break;
        }

        case vts::NodeInfo::CoveredArea::none: {
            // empty -> no children
            LOG(info3)
                << "Processed tile " << tileId
                << " (extents: " << std::fixed << node.extents()
                << ", srs: " << node.srs()
                << ") [empty].";
            return;
        }
        }

        // update upscaling level
        if (!wri.overview) {
            if (!upscaling) {
                upscaling = wri.truescale;
            } else {
                upscaling *= wri.truescale;
            }
        }
    }

    // descend to children
    descend(node, tileId, upscaling);
    // done
}

int Tiling::runImpl()
{
    auto rf(vr::system.referenceFrames(referenceFrame_));

    auto ds(geo::GeoDataset::open(dataset_));

    vts::TileIndex ti;

    TreeWalker(ti, dataset_, vts::NodeInfo(rf), lodRange_
               , asLodTileRangeList(lodRange_.min, tileRanges_)
               , tileSampling_, parallel_, forceWatertight_);

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
