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

#include <boost/thread.hpp>
#include <boost/format.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/buildsys.hpp"
#include "utility/openmp.hpp"
#include "service/cmdline.hpp"

#include "geo/geodataset.hpp"

#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/io.hpp"

#include "./tiling.hpp"

namespace fs = boost::filesystem;
namespace vts = vtslibs::vts;

namespace tiling {

namespace {

struct Flag {
    typedef vts::TileIndex::Flag::value_type value_type;
    enum : value_type {
        descend = 0x1
        , analyze = 0x2
    };
};

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
               , const Config &config);

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

    std::vector<geo::GeoDataset> gds_;
    Config config_;

    vts::TileIndex world_;
};


TreeWalker::TreeWalker(vts::TileIndex &ti, const fs::path &dataset
                       , const vts::NodeInfo &root
                       , const vts::LodRange &lodRange
                       , const vts::LodTileRange::list &tileRanges
                       , const Config &config)
    : dataset_(dataset), ti_(ti), lodRange_(lodRange)
    , config_(config)
{
    buildWorld(lodRange, tileRanges);

    if (config_.parallel) {
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

        if (config_.parallel) {
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

    int samples(config_.tileSampling);

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

            if (config_.forceWatertight
                && (res == vts::NodeInfo::CoveredArea::some))
            {
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

} // namespace

vtslibs::vts::TileIndex
generate(const fs::path &dataset
         , const vtslibs::registry::ReferenceFrame &referenceFrame
         , const vtslibs::vts::LodRange &lodRange
         , const vtslibs::vts::LodTileRange::list &tileRanges
         , const Config &config)
{
    vtslibs::vts::TileIndex ti;
    TreeWalker(ti, dataset, vts::NodeInfo(referenceFrame)
               , lodRange, tileRanges, config);
    return ti;
}

} // namespace tiling
