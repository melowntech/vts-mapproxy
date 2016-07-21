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
namespace vts = vadstena::vts;

namespace vr = vadstena::registry;

class DemTiling : public service::Cmdline {
public:
    DemTiling()
        : service::Cmdline("dem-tiling", BUILD_TARGET_VERSION)
        , extentsSampling_(20), tileSampling_(128), parallel_(true)
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
    std::string referenceFrame_;
    vts::LodRange lodRange_;
    int extentsSampling_;
    int tileSampling_;

    fs::path dataset_;
    bool parallel_;
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
        ("referenceFrame", po::value(&referenceFrame_)->required()
         , "Tiling reference frame.")
        ("lodRange", po::value(&lodRange_)->required()
         , "Lod range where content is generated.")
        ("extentsSampling", po::value(&extentsSampling_)
         ->default_value(extentsSampling_)
         , "Nuber of squares to break extents into when converting "
         "to another SRS.")
        ("tileSampling", po::value(&tileSampling_)
         ->default_value(tileSampling_)
         , "Nuber of pixels to break tile into when analyzing its coverage.")
        ("parallel", po::value(&parallel_)
         ->default_value(parallel_)
         , "Use OpenMP to parallelize work.")
        ;

    pd.add("input", 1)
        .add("referenceFrame", 1)
        ;

    (void) config;
}

void DemTiling::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    dataset_ = input_ / "dem";
    if (vars.count("output")) {
        output_ = vars["output"].as<fs::path>();
    } else {
        output_ = input_ / ("tiling." + referenceFrame_);
    }

    LOG(info3, log_)
        << "Config:"
        << "\n\tinput = " << input_
        << "\n\tdataset = " << dataset_
        << "\n\toutput = " << output_
        << "\n\treferenceFrame = " << referenceFrame_
        << "\n\tlodRange = " << lodRange_
        << "\n"
        ;
}

bool DemTiling::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("mapproxy-dem-tiling tool\n"
                "    Analyzes input dataset and generates tiling "
                "information.\n"
                "usage:\n"
                "    mapproxy-dem-tiling input referenceFrame [ options ]\n"
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
               , vts::LodRange lodRange, int tileSampling
               , bool parallel);

private:
    void process(const vts::NodeInfo &node, bool upscaling = false);

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

    const fs::path dataset_;
    vts::TileIndex &ti_;
    vts::LodRange lodRange_;
    int tileSampling_;
    geo::SrsDefinition srs_;

    std::vector<geo::GeoDataset> gds_;
    bool parallel_;
};


TreeWalker::TreeWalker(vts::TileIndex &ti, const fs::path &dataset
                       , const vts::NodeInfo &root, vts::LodRange lodRange
                       , int tileSampling, bool parallel)
    : dataset_(dataset), ti_(ti), lodRange_(lodRange)
    , tileSampling_(tileSampling), srs_(root.srsDef())
    , parallel_(parallel)
{
    if (parallel_) {
        UTILITY_OMP(parallel)
            UTILITY_OMP(single)
            {
                prepareDataset();
                process(root);
            }
    } else {
        prepareDataset();
        process(root);
    }
}

void TreeWalker::process(const vts::NodeInfo &node, bool upscaling)
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
        UTILITY_OMP(critical)
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
        if (tileId.lod >= lodRange_.min) {
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

    LOG(info2) << "Processing tile " << tileId << ".";

    int samples(upscaling ? 8 : tileSampling_);

    math::Size2 size(samples + 1, samples + 1);

    if (tileId.lod >= lodRange_.min) {
        // warp input dataset into tile
        const auto &ds(dataset());
        // set some output nodata value to force mask generation
        auto tileDs(geo::GeoDataset::deriveInMemory
                    (ds, node.srsDef(), size
                     , extentsPlusHalfPixel(node.extents(), samples)
                     , GDT_Float32
                     , geo::GeoDataset::NodataValue(-1e6)));

        auto wri([&]() -> geo::GeoDataset::WarpResultInfo
        {
            try {
                // try to warp as in mapproxy
                return ds.warpInto
                    (tileDs, geo::GeoDataset::Resampling::dem);
            } catch (const geo::WarpError &e) {
                // failed -> average could help
                LOG(info3)
                    << "Warp failed (" << e.what()
                    << "), restarting with simpler kernel.";
                return ds.warpInto
                    (tileDs, geo::GeoDataset::Resampling::average);
            }
        }());

        TiFlag::value_type baseFlags(TiFlag::mesh);
        if (!upscaling) {
            baseFlags |= TiFlag::navtile;
        }

        // grab mask of warped dataset
        const auto &mask(tileDs.cmask());
        switch (node.checkMask(mask, vts::NodeInfo::CoverageType::grid, 1)) {
        case vts::NodeInfo::CoveredArea::whole: {
            // fully covered by dataset and by reference frame definition

            if (!wri.overview) {
                // warped using original dataset, no holes -> always without
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

            UTILITY_OMP(critical)
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
            UTILITY_OMP(critical)
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

        // update upscaling flag
        upscaling = (upscaling || !wri.overview);
    }

    if (tileId.lod == lodRange_.max) {
        // no children down there
        return;
    }

    // we can proces children -> go down
    for (auto child : vts::children(tileId)) {
        // compute child node
        auto childNode(node.child(child));

        if (parallel_) {
            UTILITY_OMP(task)
                process(childNode, upscaling);
        } else {
            process(childNode, upscaling);
        }
    }

    // done
}

int DemTiling::run()
{
    auto rf(vr::system.referenceFrames(referenceFrame_));

    auto ds(geo::GeoDataset::open(dataset_));

    vts::TileIndex ti;

    TreeWalker(ti, dataset_, vts::NodeInfo(rf), lodRange_
               , tileSampling_, parallel_);

    LOG(info3) << "Saving generated tile index into " << output_ << ".";
    ti.save(output_);
    LOG(info3) << "Tile index saved.";

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    // force VRT not to share undelying datasets
    geo::Gdal::setOption("VRT_SHARED_SOURCE", 0);
    return DemTiling()(argc, argv);
}
