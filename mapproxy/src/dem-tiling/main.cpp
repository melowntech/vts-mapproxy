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
#include "utility/openmp.hpp"
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
namespace vts = vadstena::vts;

namespace vr = vadstena::registry;

class DemTiling : public service::Cmdline {
public:
    DemTiling()
        : service::Cmdline("dem-tiling", BUILD_TARGET_VERSION)
        , extentsSampling_(20), tileSampling_(128)
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
    vts::LodRange lodRange_;
    int extentsSampling_;
    int tileSampling_;
};

void DemTiling::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    vr::registryConfiguration(cmdline, vr::defaultPath());

    cmdline.add_options()
        ("dataset", po::value(&dataset_)->required()
         , "Path to dataset to proces.")
        ("output", po::value(&output_)->required()
         , "Path output tiling file.")
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
        ;

    pd.add("dataset", 1)
        .add("output", 1)
        ;

    (void) config;
}

void DemTiling::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    LOG(info3, log_)
        << "Config:"
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
        out << ("dem-tiling tool\n"
                "    Analyzes input dataset and generates tiling "
                "information.\n"
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
    TreeWalker(vts::TileIndex &ti, const std::string &dataset
               , const vts::NodeInfo &root
               , vts::LodRange lodRange, int tileSampling);

private:
    void process(const vts::NodeInfo &node, bool upscaling = false);

    const geo::GeoDataset& dataset() {
        if (!gds_.get()) {
            gds_.reset(new geo::GeoDataset(geo::GeoDataset::open(dataset_)));
        }
        return *gds_;
    }

    const std::string dataset_;
    vts::TileIndex &ti_;
    vts::LodRange lodRange_;
    int tileSampling_;
    geo::SrsDefinition srs_;

    boost::thread_specific_ptr<geo::GeoDataset> gds_;
};


TreeWalker::TreeWalker(vts::TileIndex &ti, const std::string &dataset
                       , const vts::NodeInfo &root, vts::LodRange lodRange
                       , int tileSampling)
    : dataset_(dataset), ti_(ti), lodRange_(lodRange)
    , tileSampling_(tileSampling), srs_(root.srsDef())
{
    UTILITY_OMP(parallel)
        UTILITY_OMP(single)
        process(root);
}

void TreeWalker::process(const vts::NodeInfo &node, bool upscaling)
{
    const auto tileId(node.nodeId());
    if (!node.valid() || (tileId.lod > lodRange_.max)) {
        // invalid node
        return;
    }

    LOG(info2) << "Processing tile " << tileId << ".";

    int samples(upscaling ? 8 : tileSampling_);

    math::Size2 size(samples + 1, samples + 1);

    if (tileId.lod >= lodRange_.min) {
        // warp input dataset into tile
        const auto& ds(dataset());
        auto tileDs(geo::GeoDataset::deriveInMemory
                    (ds, node.srsDef(), size
                     , extentsPlusHalfPixel(node.extents(), samples)
                     , GDT_Float64));
        geo::GeoDataset::WarpOptions wo;

        // set some output nodata value to force mask generation
        wo.dstNodataValue = std::numeric_limits<double>::lowest();
        auto wri(ds.warpInto(tileDs, geo::GeoDataset::Resampling::cubicspline
                             , wo));

        TiFlag::value_type baseFlags(TiFlag::mesh);
        if (!upscaling) {
            baseFlags |= TiFlag::navtile;
        }

        // grab mask of warped dataset
        const auto mask(tileDs.cmask());
        if (mask.full() && !node.partial()) {
            // fully covered by dataset and by reference frame definition

            if (!wri.overview) {
                // warped using original dataset, no holes -> always without
                // holes -> set whole subtree to watertight mesh

                UTILITY_OMP(critical)
                {
                    ti_.set(vts::LodRange(tileId.lod, lodRange_.max)
                            , vts::tileRange(tileId)
                            , (TiFlag::mesh | TiFlag::watertight));
                }

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
        } else if (!mask.empty()) {
            // partially covered
            UTILITY_OMP(critical)
                ti_.set(tileId, baseFlags);
            LOG(info3)
                << "Processed tile " << tileId
                << " (extents: " << std::fixed << node.extents()
                << ", srs: " << node.srs()
                << ") [partial].";
        } else {
            // empty -> no children
            LOG(info3)
                << "Processed tile " << tileId
                << " (extents: " << std::fixed << node.extents()
                << ", srs: " << node.srs()
                << ") [empty].";
            return;
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

        UTILITY_OMP(task)
            process(childNode, upscaling);
    }

    // done
}

int DemTiling::run()
{
    auto rf(vr::Registry::referenceFrame(referenceFrame_));

    auto ds(geo::GeoDataset::open(dataset_));

    vts::TileIndex ti;

    TreeWalker(ti, dataset_, vts::NodeInfo(rf), lodRange_, tileSampling_);

    LOG(info3) << "Saving generated tile index into " << output_ << ".";
    ti.save(output_);
    LOG(info3) << "Tile index saved.";

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    return DemTiling()(argc, argv);
}
