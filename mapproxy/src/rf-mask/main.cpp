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
#include "utility/progress.hpp"
#include "utility/path.hpp"
#include "service/cmdline.hpp"

#include "imgproc/rastermask/mappedqtree.hpp"

#include "geo/geodataset.hpp"

#include "vts-libs/registry/po.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/tileindex.hpp"

#include "./ogrsupport.hpp"

#include "gdal-drivers/mask.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace vts = vadstena::vts;

namespace vr = vadstena::registry;

class RfMask : public service::Cmdline {
public:
    RfMask()
        : service::Cmdline("rf-mask", BUILD_TARGET_VERSION)
        , dilate_(10.), segments_(30)
        , tileSizeOrder_(8), workBlock_(5)
        , generateGdalDatasets_(false)
    {
    }

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    void rasterize(imgproc::quadtree::RasterMask &mask
                   , const vts::NodeInfo &node, ::OGRLayer *layer);

    std::string dataset_;
    fs::path output_;
    std::string referenceFrame_;
    std::string layer_;
    double dilate_;
    int segments_;
    int lod_;

    int tileSizeOrder_;
    math::Size2 tileSize_;
    int workBlock_;
    bool generateGdalDatasets_;
};

void RfMask::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    vr::registryConfiguration(config, vr::defaultPath());

    cmdline.add_options()
        ("dataset", po::value(&dataset_)->required()
         , "Path to ORG dataset to proces.")
        ("output", po::value(&output_)->required()
         , "Path output mask file.")
        ("referenceFrame", po::value(&referenceFrame_)->required()
         , "Reference frame.")

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
        ;

    pd.add("dataset", 1)
        .add("output", 1)
        ;

    (void) config;
}

void RfMask::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    tileSize_ = math::Size2(1 << tileSizeOrder_, 1 <<  tileSizeOrder_);

    LOG(info3, log_)
        << "Config:"
        << "\n\tdataset = " << dataset_
        << "\n\toutput = " << output_
        << "\n\treferenceFrame = " << referenceFrame_
        << "\n\tlod = " << lod_
        << "\n\tdilate = " << dilate_
        << "\n\tsegments = " << segments_
        << "\n\ttileSize = " << tileSize_
        << "\n\tworkBlock = " << workBlock_
        << "\n\tgenerateGdalDatasets = " << generateGdalDatasets_
        << "\n"
        ;
}

bool RfMask::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("rf-mask tool\n"
                "    Converts OGR dataset into quadtree-represented mask "
                "used by mapproxy.\n"
                "\n"
                );

        return true;
    }

    return false;
}

inline ::OGRRawPoint rawPoint(double x, double y)
{
    ::OGRRawPoint p;
    p.x = x;
    p.y = y;
    return p;
}

geo::Geometries
translateDilateClip(::OGRLayer *layer, const geo::SrsDefinition &srs
                    , double dilate, int segments
                    , const math::Extents2 &extents)
{
    auto srsRef(srs.reference());

    geo::Geometries geometries;

    OGR_L_ResetReading(layer);

    while (auto f = geo::feature(OGR_L_GetNextFeature(layer))) {
        auto g(static_cast< ::OGRGeometry*>(OGR_F_GetGeometryRef(f.get())));
        if (!g) { continue; }

        auto ng(geo::geometry(g->clone()));
        ng->transformTo(&srsRef);

        // remember dilated geometry
        if (!dilate) {
            // use original geometry if dilate distance is zero
            geometries.push_back(ng);
        } else {
            geometries.push_back
                (geo::geometry(ng->Buffer(dilate, segments)));
        }
    }

    // clip polygon from extents
    ::OGRRawPoint clipPoints[5] = {
        rawPoint(extents.ll(0), extents.ll(1))
        , rawPoint(extents.ll(0), extents.ur(1))
        , rawPoint(extents.ur(0), extents.ur(1))
        , rawPoint(extents.ur(0), extents.ll(1))
        , rawPoint(extents.ll(0), extents.ll(1))
    };
    ::OGRLinearRing clipRing;
    clipRing.setPoints(5, clipPoints);
    ::OGRPolygon clip;
    clip.addRing(&clipRing);

    geo::Geometries out;
    for (const auto &g : geometries) {
        auto gc(geo::geometry(g->Intersection(&clip)));
        if (!gc->IsEmpty()) {
            out.push_back(gc);
        }
    }
    return out;
}

struct TIDGuard {
    TIDGuard(const std::string &id)
        : old(dbglog::thread_id())
    {
        dbglog::thread_id(id);
    }
    ~TIDGuard() { dbglog::thread_id(old); }

    const std::string old;
};

class Tiling {
public:
    Tiling(const math::Extents2 &rootExtents, int lod
           , int block, const math::Extents2 &extents);


    const math::Extents2& extents() const { return extents_; }

    const math::Size2& blocks() const { return blocks_; }

    const math::Size2& tileCount() const { return tileCount_; }

    math::Extents2 blockExtents(const math::Point2i &pos) const;

    math::Extents2i blockTiles(const math::Point2i &pos) const;

    /** void Op(const math::Point2i &block, const math::Extents2 &extents
     *          , const math::Point2i &tile)
     */
    template <typename Op> void forEachBlock(Op op) const {
        int total(area(blocks_));

        utility::ts::Progress
            progress("Processed blocks", total
                     , utility::ts::Progress::Ratio(1, 1000));

        UTILITY_OMP(parallel for schedule(dynamic))
        for (int bi = 0; bi < total; ++bi) {
            auto i(bi % blocks_.width);
            auto j(bi / blocks_.width);

            {
                TIDGuard tid(str(boost::format("Block [%d, %d]") % i % j));
                math::Point2i block(i, j);
                op(blockOrigin_ + block, blockExtents(block));
            }

            ++progress;
        }
    }

private:
    math::Point2 origin_;
    math::Size2 block_;
    math::Size2f tileSize_;
    math::Extents2 extents_;
    math::Size2 blocks_;
    math::Size2f blockSize_;
    math::Point2i blockOrigin_;
    math::Size2 tileCount_;
    math::Extents2i tiles_;
};

Tiling::Tiling(const math::Extents2 &rootExtents, int lod
               , int block, const math::Extents2 &extents)
    : origin_(ul(rootExtents))
    , block_(1 << block, 1 << block)
{
    auto rs(size(rootExtents));
    auto tc(1 << lod);
    tileSize_.width = rs.width / tc;
    tileSize_.height = rs.height / tc;
    blockSize_.width = block_.width * tileSize_.width;
    blockSize_.height = block_.height * tileSize_.height;

    // calculate local extents from root ul(extents)
    math::Extents2 local(extents.ll(0) - origin_(0)
                         , origin_(1) - extents.ur(1)
                         , extents.ur(0) - origin_(0)
                         , origin_(1) - extents.ll(1));

    tiles_ = { int(std::floor(local.ll(0) / tileSize_.width))
               , int(std::floor(local.ll(1) / tileSize_.height))
               , int(std::ceil(local.ur(0) / tileSize_.width))
               , int(std::ceil(local.ur(1) / tileSize_.height)) };

    // clamp extents to block grid
    auto mask((1 << block) - 1);
    tiles_.ll(0) &= ~mask;
    tiles_.ll(1) &= ~mask;
    tiles_.ur(0) += mask;
    tiles_.ur(1) += mask;
    tiles_.ur(0) &= ~mask;
    tiles_.ur(1) &= ~mask;

    extents_ = { origin_(0) + tiles_.ll(0) * tileSize_.width
                 , origin_(1) - tiles_.ur(1) * tileSize_.height
                 , origin_(0) + tiles_.ur(0) * tileSize_.width
                 , origin_(1) - tiles_.ll(1) * tileSize_.height };

    tileCount_ = size(tiles_);
    blocks_.width = ((tileCount_.width + block_.width - 1) / block_.width);
    blocks_.height = ((tileCount_.height + block_.height - 1) / block_.height);

    blockOrigin_ = ll(tiles_);
    blockOrigin_(0) >>= block;
    blockOrigin_(1) >>= block;

    LOG(info3) << std::fixed << "Input extents: " << extents;
    LOG(info3) << std::fixed << "Blocked extents: " << extents_;
    LOG(info3) << std::fixed << "Origin: " << origin_;
    LOG(info3) << "Tiles: " << tiles_;
    LOG(info3) << "Tile count: " << tileCount_;
    LOG(info3) << "Block origin: " << blockOrigin_;
    LOG(info3) << "Block count: " << blocks_;
}

math::Extents2 Tiling::blockExtents(const math::Point2i &pos) const
{
    math::Point2 ul
        (origin_(0) + (blockOrigin_(0) + pos(0)) * blockSize_.width
         , origin_(1) - (blockOrigin_(1) + pos(1)) * blockSize_.height);
    return { ul(0), ul(1) - blockSize_.height
            , ul(0) + blockSize_.width, ul(1) };
}

math::Extents2i Tiling::blockTiles(const math::Point2i &pos) const
{
    math::Point2i ul(pos(0) * block_.width, pos(1) * block_.height);
    return { ul(0), ul(1), ul(0) + block_.width
            , ul(1) + block_.height };
}

math::Extents2 geometryExtents(const geo::Geometry &geometry)
{
    OGREnvelope e;
    geometry->getEnvelope(&e);

    return math::Extents2(e.MinX, e.MinY, e.MaxX, e.MaxY);
}

math::Extents2 geometryExtents(const geo::Geometries &geometries)
{
    math::Extents2 e(math::InvalidExtents{});
    for (const auto &g : geometries) {
        auto ee(geometryExtents(g));
        update(e, ee.ll);
        update(e, ee.ur);
    }

    return e;
}

void rasterizeBlock(const math::Point2i &block
                    , const math::Extents2 &extents
                    , imgproc::quadtree::RasterMask &mask
                    , const vts::TileId &blockId
                    , const vts::TileId &tileReference
                    , const geo::SrsDefinition &srs
                    , const geo::Geometries &geometries
                    , const math::Size2 &blockSize
                    , const math::Size2 &tileSize)
{
    LOG(info2)
        << std::fixed << "Rasterizing block (" << block(0) << ", "
        << block(1) << ") (extents: " << extents
        << ", blockId: " << blockId << ").";

    // single byte channel dataset in memory without no-data value
    auto ds(geo::GeoDataset::create
            ("", srs, extents
             , { blockSize.width * tileSize.width
                    , blockSize.height * tileSize.height}
             , geo::GeoDataset::Format::coverage
             (geo::GeoDataset::Format::Storage::memory)
             , geo::NodataValue(0)));

    // clip polygon by block extents
    ::OGRRawPoint clipPoints[5] = {
        rawPoint(extents.ll(0), extents.ll(1))
        , rawPoint(extents.ll(0), extents.ur(1))
        , rawPoint(extents.ur(0), extents.ur(1))
        , rawPoint(extents.ur(0), extents.ll(1))
        , rawPoint(extents.ll(0), extents.ll(1))
    };
    ::OGRLinearRing clipRing;
    clipRing.setPoints(5, clipPoints);
    ::OGRPolygon clip;
    clip.addRing(&clipRing);

    std::size_t emptyCount(0);
    bool full(false);
    for (const auto &g : geometries) {
        auto gc(geo::geometry(g->Intersection(&clip)));
        if (gc->IsEmpty()) {
            ++emptyCount;
        } else if (gc->Equals(&clip)) {
            full = true;
            break;
        } else {
            ds.rasterize(gc.get(), geo::BurnColor(255.0));
        }
    }

    if (emptyCount == geometries.size()) {
        return;
    } else if (full) {
        UTILITY_OMP(critical)
            mask.setQuad(blockId.lod, blockId.x, blockId.y);
        return;
    }

    // fetch mask layer from dataset
    auto maskLayer(ds.fetchMask());
    auto tileArea(math::area(tileSize));
    for (int j(0), je(blockSize.height); j != je; ++j) {
        for (int i(0), ie(blockSize.width); i != ie; ++i) {
            // get tile from mask layer
            cv::Rect rect(i * tileSize.width, j * tileSize.height
                          , tileSize.height, tileSize.height);
            cv::Mat tile(maskLayer, rect);

            auto nz(countNonZero(tile));
            if (!nz) {
                // empty
                continue;
            }

            vts::TileId tileId(tileReference.lod, tileReference.x + i
                               , tileReference.y + j);

            if (nz == tileArea) {
                // full
                UTILITY_OMP(critical)
                    mask.setQuad(tileId.lod, tileId.x, tileId.y);
                continue;
            }

            // partial, need to generate mask
            const auto *d(tile.data);
            if (nz > (tileArea / 2)) {
                // at least half is set, start with full and remove unset pixels
                imgproc::quadtree::RasterMask m
                    (tileSize.width, tileSize.height
                     , imgproc::quadtree::RasterMask::FULL);
                for (int jj = 0; jj < tileSize.height; ++jj) {
                    for (int ii = 0; ii < tileSize.width; ++ii) {
                        if (!*d++) {
                            m.set(ii, jj, false);
                        }
                    }
                }

                UTILITY_OMP(critical)
                    mask.setSubtree(tileId.lod, tileId.x, tileId.y, m);
            } else {
                // less than half is set, start with empty and set pixels
                imgproc::quadtree::RasterMask m
                    (tileSize.width, tileSize.height
                     , imgproc::quadtree::RasterMask::EMPTY);
                for (int jj = 0; jj < tileSize.height; ++jj) {
                    for (int ii = 0; ii < tileSize.width; ++ii) {
                        if (*d++) {
                            m.set(ii, jj, true);
                        }
                    }
                }

                UTILITY_OMP(critical)
                    mask.setSubtree(tileId.lod, tileId.x, tileId.y, m);
            }
        }
    }
    const auto &cm(ds.cmask(true));
    UTILITY_OMP(critical)
        mask.setSubtree(blockId.lod, blockId.x, blockId.y, cm);
}

void RfMask::rasterize(imgproc::quadtree::RasterMask &mask
                       , const vts::NodeInfo &node, ::OGRLayer *layer)
{
    LOG(info3) << "Converting input into <" << node.srs() << ">.";

    const auto srs(node.srsDef());
    auto geometries(translateDilateClip(layer, srs, dilate_, segments_
                                        , node.extents()));
    auto ge(geometryExtents(geometries));
    if (!valid(ge)) {
        LOG(info3)
            << "No data in node " << node.nodeId()
            << " (srs: " << node.srs() << ").";
        return;
    }

    LOG(info3) << "Converted " << geometries.size()
               << " geometries into <" << node.srs() << "> and dilated by "
               << dilate_ << " units.";

    const math::Size2 blockSize((1 << workBlock_), (1 << workBlock_));

    // NB: local lod!
    Tiling tiling(node.extents(), lod_ - node.nodeId().lod, workBlock_, ge);

    // reference tile in grid
    auto reference
        (vts::lowestChild(node.nodeId()
                          , (lod_ - workBlock_ - node.nodeId().lod)));

    tiling.forEachBlock([&](const math::Point2i &block
                            , const math::Extents2 &extents)
    {
        vts::TileId blockId(reference.lod, reference.x + block(0)
                            , reference.x + block(1));
        auto tileReference(vts::lowestChild(blockId, tileSizeOrder_));

        rasterizeBlock(block, extents, mask, blockId, tileReference
                       , srs, geometries, blockSize, tileSize_);
    });
}

int RfMask::run()
{
    auto rf(vr::Registry::referenceFrame(referenceFrame_));

    LOG(info3) << "Opening " << dataset_ << ".";
    auto in(geo::openVectorDataset(dataset_));

    if (!::OGR_DS_GetLayerCount(in.get())) {
        LOGTHROW(err2, std::runtime_error)
            << "Dataset has no layer.";
    } else if (layer_.empty() && (::OGR_DS_GetLayerCount(in.get()) > 1)) {
        LOGTHROW(err2, std::runtime_error)
            << "There is more than one layer; tell me which one you want.";
    }

    auto inLayer(static_cast< ::OGRLayer*>
                 (layer_.empty()
                  ? ::OGR_DS_GetLayer(in.get(), 0)
                  : ::OGR_DS_GetLayerByName(in.get(), layer_.c_str())));

    if (!inLayer) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot get layer from input dataset.";
    }

    auto srs(geo::SrsDefinition::fromReference
             (*inLayer->GetSpatialRef(), geo::SrsDefinition::Type::wkt));

    // prepare raster mask with proper depth
    imgproc::quadtree::RasterMask mask
        (1 << (lod_ + tileSizeOrder_), 1 << (lod_ + tileSizeOrder_)
         , imgproc::quadtree::RasterMask::InitMode::EMPTY);

    // process only leaf nodes of the reference frame
    for (const auto &item : rf.division.nodes) {
        const auto &id(item.first);
        vts::NodeInfo node(rf, id);
        if (!node.valid()) { continue; }

        // process only leaf nodes of the reference frame tree
        bool leaf(true);
        for (const auto &item2 : rf.division.nodes) {
            const auto &id2(item2.first);
            if (id2 == id) { continue; }
            vts::NodeInfo node2(rf, id);
            if ((id2.lod > id.lod) && (node2.subtree().id() == id)) {
                leaf = false;
                break;
            }
        }

        if (!leaf) { continue; }

        rasterize(mask, node, inLayer);

        if (generateGdalDatasets_) {
            const auto ext(str(boost::format(".%s.mask") % id));
            auto path(utility::addExtension(output_, ext));
            LOG(info3)
                << "Saving subtree " << id << " (" << node.srs()
                << ") into file " << path << ".";

            // save generated subtree as gdal dataset
            gdal_drivers::MaskDataset::create
                (path, mask, node.extents(), node.srsDef()
                 , id.lod, id.x, id.y);
        }
    }

    {
        LOG(info3)
            << "Saving output to " << output_ << ".";
        utility::ofstreambuf f(output_.string());
        imgproc::mappedqtree::RasterMask::write(f, mask);
        f.close();
    }

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    ::OGRRegisterAll();
    return RfMask()(argc, argv);
}
