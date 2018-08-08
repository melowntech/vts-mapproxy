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

#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"
#include "utility/format.hpp"
#include "utility/path.hpp"

#include "geo/geodataset.hpp"

#include "imgproc/rastermask/cvmat.hpp"
#include "imgproc/png.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/tileindex.hpp"

#include "../error.hpp"
#include "../support/metatile.hpp"
#include "../support/tileindex.hpp"
#include "../support/mmapped/qtree.hpp"
#include "../support/mmapped/qtree-rasterize.hpp"
#include "../support/revision.hpp"

#include "./tms-raster.hpp"
#include "./factory.hpp"
#include "../support/python.hpp"

#include "browser2d/index.html.hpp"

namespace fs = boost::filesystem;
namespace bgil = boost::gil;
namespace vr = vtslibs::registry;

namespace generator {

namespace {

/** NOTICE: increment each time some data-related bug is fixed.
 */
int GeneratorRevision(2);

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<TmsRaster>(params);
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType<TmsRaster>(std::make_shared<Factory>());
});

inline boost::optional<fs::path>
asPath(const boost::optional<std::string> &path)
{
    if (!path) { return boost::none; }
    return fs::path(*path);
}

} // namespace

TmsRaster::TmsRaster(const Params &params)
    : Generator(params)
    , definition_(resource().definition<Definition>())
    , hasMetatiles_(false)
    , complexDataset_(false)
    , maskTree_(ignoreNonexistent(absoluteDatasetRf(asPath(definition_.mask))))
{
    const auto indexPath(root() / "tileset.index");
    const auto deliveryIndexPath(root() / "delivery.index");

    if (changeEnforced()) {
        LOG(info1) << "Generator for <" << id() << "> not ready.";
        return;
    }

    if (fs::exists(deliveryIndexPath)) {
        index_ = boost::in_place(deliveryIndexPath);
    }

    if (index_) {
        hasMetatiles_ = true;
        complexDataset_
            = fs::exists(absoluteDataset(definition_.dataset + "/ophoto"));
        makeReady();
        return;
    };

    // not seen or index-less
    LOG(info1) << "Generator for <" << id() << "> not ready.";
}

TmsRaster::DatasetDesc TmsRaster::dataset_impl() const
{
    if (complexDataset_) {
        return { definition_.dataset + "/ophoto" };
    }

    return { definition_.dataset };
}

void TmsRaster::prepare_impl(Arsenal&)
{
    LOG(info2) << "Preparing <" << id() << ">.";

    if (fs::exists(absoluteDataset(definition_.dataset + "/ophoto"))) {
        // complex dataset directory
        // alwas with metatiles
        hasMetatiles_ = true;
        complexDataset_ = true;

        // try to open
        geo::GeoDataset::open
            (absoluteDataset(definition_.dataset + "/ophoto"));

        const auto &r(resource());

        vts::TileIndex index;
        prepareTileIndex(index
                         , (absoluteDataset(definition_.dataset)
                            + "/tiling." + r.id.referenceFrame)
                         , r, false, maskTree_);

        // save vts::TileIndex anyway
        index.save(root() / "tileset.index");

        // store and open
        const auto deliveryIndexPath(root() / "delivery.index");
        const auto tmpPath(utility::addExtension(deliveryIndexPath, ".tmp"));
        mmapped::TileIndex::write(tmpPath, index);
        fs::rename(tmpPath, deliveryIndexPath);
        index_ = boost::in_place(deliveryIndexPath);

        // done
        makeReady();
        return;
    }

    // try to open datasets
    auto ds(geo::GeoDataset::open(absoluteDataset(dataset().path)));
    if (maskTree_) {
        // we have mask tree -> metatiles exist
        hasMetatiles_ = true;

        // build tileindex
        vts::TileIndex index;
        prepareTileIndex(index, resource(), false, maskTree_);

        // save vts::TileIndex anyway
        index.save(root() / "tileset.index");

        // store and open
        const auto deliveryIndexPath(root() / "delivery.index");
        mmapped::TileIndex::write(deliveryIndexPath, index);
        index_ = boost::in_place(deliveryIndexPath);
    } else if (definition_.mask) {
        maskDataset_ = definition_.mask;
        geo::GeoDataset::open(absoluteDataset(*maskDataset_));
        // we have mask dataset -> metatiles exist
        hasMetatiles_ = true;
    } else {
        // no external mask available -> metatiles exist only when dataset has
        // some invalid pixels
        hasMetatiles_ = !ds.allValid();
    }

    makeReady();
}

RasterFormat TmsRaster::format() const
{
    return transparent() ? RasterFormat::png : definition_.format;
}

bool TmsRaster::transparent_impl() const
{
    return definition_.transparent;
}

bool TmsRaster::hasMask_impl() const
{
    // we want mask by default
    return true;
}

vr::BoundLayer TmsRaster::boundLayer(ResourceRoot root) const
{
    const auto &res(resource());

    vr::BoundLayer bl;
    bl.id = res.id.fullId();
    bl.numericId = 0; // no numeric ID
    bl.type = vr::BoundLayer::Type::raster;

    // build url
    bl.url = prependRoot
        (utility::format("{lod}-{x}-{y}.%s%s"
                         , format(), RevisionWrapper(res.revision, "?"))
         , resource(), root);
    if (hasMask()) {
        bl.maskUrl = prependRoot
            (utility::format("{lod}-{x}-{y}.mask%s"
                             , RevisionWrapper(res.revision, "?"))
             , resource(), root);
        if (hasMetatiles_) {
            const auto fname
                (utility::format("{lod}-{x}-{y}.meta?gr=%d%s"
                                 , GeneratorRevision
                                 , RevisionWrapper(res.revision, "&")));

            bl.metaUrl = prependRoot(fname, resource(), root);
        }
    }

    bl.lodRange = res.lodRange;
    bl.tileRange = res.tileRange;
    bl.credits = asInlineCredits(res);
    bl.isTransparent = transparent();

    // done
    return bl;
}

vts::MapConfig TmsRaster::mapConfig_impl(ResourceRoot root)
    const
{
    const auto &res(resource());

    vts::MapConfig mapConfig;
    mapConfig.referenceFrame = *res.referenceFrame;

    // this is Tiled service: we have bound layer only; use remote definition
    mapConfig.boundLayers.add
        (vr::BoundLayer
         (res.id.fullId()
          , prependRoot(std::string("boundlayer.json"), resource(), root)));

    return mapConfig;
}

Generator::Task TmsRaster::generateFile_impl(const FileInfo &fileInfo
                                             , Sink &sink) const
{
    TmsFileInfo fi(fileInfo);

    // check for valid tileId
    switch (fi.type) {
    case TmsFileInfo::Type::image:
    case TmsFileInfo::Type::mask:
        if (!checkRanges(resource(), fi.tileId)) {
            sink.error(utility::makeError<NotFound>
                        ("TileId outside of configured range."));
            return {};
        }
        break;

    case TmsFileInfo::Type::metatile:
        if (!hasMetatiles_) {
            sink.error(utility::makeError<NotFound>
                        ("This dataset doesn't provide metatiles."));
            return {};
        }
        if (!checkRanges(resource(), fi.tileId, RangeType::lod)) {
            sink.error(utility::makeError<NotFound>
                        ("TileId outside of configured range."));
            return {};
        }
        break;

    default: break;
    }

    switch (fi.type) {
    case TmsFileInfo::Type::unknown:
        sink.error(utility::makeError<NotFound>("Unrecognized filename."));
        break;

    case TmsFileInfo::Type::config: {
        std::ostringstream os;
        mapConfig(os, ResourceRoot::none);
        sink.content(os.str(), fi.sinkFileInfo());
        break;
    };

    case TmsFileInfo::Type::definition: {
        std::ostringstream os;
        vr::saveBoundLayer(os, boundLayer(ResourceRoot::none));
        sink.content(os.str(), fi.sinkFileInfo());
        break;
    }

    case TmsFileInfo::Type::support:
        sink.content(fi.support->data, fi.support->size
                      , fi.sinkFileInfo(), false);
        break;

    case TmsFileInfo::Type::image: {
        if (fi.format != format()) {
            sink.error(utility::makeError<NotFound>
                        ("Format <%s> is not supported by this resource (%s)."
                         , fi.format, format()));
            return {};
        }

        return[=](Sink &sink, Arsenal &arsenal) {
            generateTileImage(fi.tileId, fi, sink, arsenal);
        };
    }

    case TmsFileInfo::Type::mask:
        if (maskTree_) {
            return [=](Sink &sink, Arsenal &arsenal)  {
                generateTileMaskFromTree(fi.tileId, fi, sink, arsenal);
            };
        } else {
            return [=](Sink &sink, Arsenal &arsenal)  {
                generateTileMask(fi.tileId, fi, sink, arsenal);
            };
        }

    case TmsFileInfo::Type::metatile:
        return [=](Sink &sink, Arsenal &arsenal) {
            generateMetatile(fi.tileId, fi, sink, arsenal);
        };
    }

    return {};
}

void TmsRaster::generateTileImage(const vts::TileId &tileId
                                  , const TmsFileInfo &fi
                                  , Sink &sink
                                  , Arsenal &arsenal) const
{
    sink.checkAborted();

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        sink.error(utility::makeError<NotFound>
                    ("TileId outside of valid reference frame tree."));
        return;
    }

    if (!nodeInfo.productive()
        || (index_ && !vts::TileIndex::Flag::isReal(index_->get(tileId))))
    {
        sink.error(utility::makeError<EmptyImage>("No valid data."));
        return;
    }

    // grab dataset to use
    const auto ds(dataset());

    // what operation should we do with the dataset? optimized or non-optimized
    // return?
    //
    // * dynamic dataset: cannot use since we are unable report different
    //                    caching for empty/full image
    // * transparent: we cannot report transparecny for empty/full image
    const auto operation
        ((ds.dynamic || transparent())
         ? GdalWarper::RasterRequest::Operation::imageNoOpt
         : GdalWarper::RasterRequest::Operation::image);

    // choose resampling (configured or default)
    const auto resampling(definition_.resampling ? *definition_.resampling
                          : geo::GeoDataset::Resampling::cubic);

    auto tile(arsenal.warper.warp
              (GdalWarper::RasterRequest
               (operation
                , absoluteDataset(ds.path)
                , nodeInfo.srsDef()
                , nodeInfo.extents()
                , math::Size2(256, 256)
                , resampling
                , absoluteDataset(maskDataset_))
               , sink));
    sink.checkAborted();

    // serialize
    std::vector<unsigned char> buf;
    switch (fi.format) {
    case RasterFormat::jpg:
        // TODO: configurable quality
        cv::imencode(".jpg", *tile, buf
                     , { cv::IMWRITE_JPEG_QUALITY, 75 });
        break;

    case RasterFormat::png:
        cv::imencode(".png", *tile, buf
                     , { cv::IMWRITE_PNG_COMPRESSION, 9 });
        break;
    }

    sink.content(buf, fi.sinkFileInfo().setMaxAge(ds.maxAge));
}

void TmsRaster::generateTileMask(const vts::TileId &tileId
                                 , const TmsFileInfo &fi
                                 , Sink &sink
                                 , Arsenal &arsenal) const
{
    sink.checkAborted();

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        sink.error(utility::makeError<NotFound>
                    ("TileId outside of valid reference frame tree."));
        return;
    }

    if (!nodeInfo.productive()
        || (index_ && !vts::TileIndex::Flag::isReal(index_->get(tileId))))
    {
        sink.error(utility::makeError<EmptyImage>("No valid data."));
        return;
    }

    // get dataset
    auto ds(dataset());

    auto mask(arsenal.warper.warp
              (GdalWarper::RasterRequest
               (GdalWarper::RasterRequest::Operation::mask
                , absoluteDataset(ds.path, maskDataset_)
                , nodeInfo.srsDef()
                , nodeInfo.extents()
                , math::Size2(256, 256)
                , geo::GeoDataset::Resampling::cubic)
               , sink));

    sink.checkAborted();

    // serialize
    std::vector<unsigned char> buf;
    // write as png file
    cv::imencode(".png", *mask, buf
                 , { cv::IMWRITE_PNG_COMPRESSION, 9 });

    // reset max age received from dataset if mask is used
    if (maskDataset_) { ds.maxAge = boost::none; }
    sink.content(buf, fi.sinkFileInfo().setMaxAge(ds.maxAge));
}

void TmsRaster::generateTileMaskFromTree(const vts::TileId &tileId
                                         , const TmsFileInfo &fi
                                         , Sink &sink
                                         , Arsenal&) const
{
    sink.checkAborted();

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        sink.error(utility::makeError<NotFound>
                    ("TileId outside of valid reference frame tree."));
        return;
    }

    if (!nodeInfo.productive()
        || (index_ && !vts::TileIndex::Flag::isReal(index_->get(tileId))))
    {
        sink.error(utility::makeError<EmptyImage>("No valid data."));
        return;
    }

    auto mask(boundlayerMask(tileId, maskTree_));

    const auto nz(countNonZero(mask));
    if (!nz) {
        sink.error(utility::makeError<EmptyImage>
                   ("No pixels, optimize."));
    } else if (nz == vr::BoundLayer::basicTileArea) {
        sink.error(utility::makeError<FullImage>
                   ("All pixels valid, optimize."));
    }

    // serialize
    std::vector<unsigned char> buf;
    // write as png file
    cv::imencode(".png", mask, buf
                 , { cv::IMWRITE_PNG_COMPRESSION, 9 });

    sink.content(buf, fi.sinkFileInfo());
}

namespace Constants {
    const unsigned int RasterMetatileBinaryOrder(8);
    const math::Size2 RasterMetatileSize(1 << RasterMetatileBinaryOrder
                                         , 1 << RasterMetatileBinaryOrder);
}

namespace MetaFlags {
    constexpr std::uint8_t watertight(0xc0);
    constexpr std::uint8_t available(0x80);
    constexpr std::uint8_t unavailable(0x00);
}

namespace {

void meta2d(const mmapped::TileIndex &tileIndex, const vts::TileId &tileId
            , const TmsFileInfo &fi, Sink &sink)
{
    bgil::gray8_image_t out(Constants::RasterMetatileSize.width
                            , Constants::RasterMetatileSize.height
                            , bgil::gray8_pixel_t(0x00), 0);
    auto outView(view(out));

    if (const auto *tree = tileIndex.tree(tileId.lod)) {
        const auto parentId
            (vts::parent(tileId, Constants::RasterMetatileBinaryOrder));

        rasterize(*tree, parentId.lod, parentId.x, parentId.y
                  , outView, [&](vts::QTree::value_type flags) -> std::uint8_t
        {
            std::uint8_t out(0);

            if (flags & vts::TileIndex::Flag::mesh) {
                out |= MetaFlags::available;

                if (flags & vts::TileIndex::Flag::watertight) {
                    out |= MetaFlags::watertight;
                }
            }

            return out;
        });
    }

    sink.content(imgproc::png::serialize(out, 9), fi.sinkFileInfo());
}

template <typename SrcType>
void fillMetatile(cv::Mat &metatile, const cv::Mat &src
                  , const math::Point2i &origin
                  , const math::Point2i &end
                  , std::uint8_t watertightBits)
{
    for (int j(origin(1)), je(end(1)), jj(0); j <= je; ++j, ++jj) {
        for (int i(origin(0)), ie(end(0)), ii(0); i <= ie; ++i, ++ii) {
            const auto &in(src.at<SrcType>(jj, ii));
            auto &out(metatile.at<std::uint8_t>(j, i));
            if (!in) {
                // black -> not available
                out = MetaFlags::unavailable;
            } else if (in >= 255) {
                // white -> available and watertight
                // -> use provided watertight bits
                out = watertightBits;
            } else {
                // gray -> available but not watetight
                out = MetaFlags::available;
            }
        }
    }
}

} // namespace

void TmsRaster::generateMetatile(const vts::TileId &tileId
                                 , const TmsFileInfo &fi
                                 , Sink &sink
                                 , Arsenal &arsenal) const
{
    sink.checkAborted();

    auto blocks(metatileBlocks
                (resource(), tileId, Constants::RasterMetatileBinaryOrder));

    if (blocks.empty()) {
        sink.error(utility::makeError<NotFound>
                    ("Metatile completely outside of configured range."));
        return;
    }

    if (index_) {
        // render tileindex
        meta2d(*index_, tileId, fi, sink);
        return;
    }

    // non-tileindex code
    cv::Mat metatile(Constants::RasterMetatileSize.width
                     , Constants::RasterMetatileSize.height
                     , CV_8U, cv::Scalar(0));

    auto ds(dataset());

    for (const auto &block : blocks) {
        if (!block.commonAncestor.productive()) { continue; }

        const auto &view(block.view);
        math::Size2 bSize(vts::tileRangesSize(view));

        GdalWarper::Raster src;
        if (maskDataset_) {
            // warp detailed mask
            src = arsenal.warper.warp
                (GdalWarper::RasterRequest
                 (GdalWarper::RasterRequest::Operation::detailMask
                  , absoluteDataset(*maskDataset_)
                  , vr::system.srs(block.srs).srsDef
                  , block.extents, bSize)
                 , sink);
        } else {
            // warp dataset as mask
            src = arsenal.warper.warp
                (GdalWarper::RasterRequest
                 (GdalWarper::RasterRequest::Operation::maskNoOpt
                  , absoluteDataset(ds.path)
                  , vr::system.srs(block.srs).srsDef
                  , block.extents, bSize
                  , geo::GeoDataset::Resampling::average)
                 , sink);
        }
        sink.checkAborted();

        // generate metatile content for current block
        math::Point2i origin(view.ll(0) - tileId.x, view.ll(1) - tileId.y);
        math::Point2i end(view.ur(0) - tileId.x, view.ur(1) - tileId.y);

        if (maskDataset_) {
            // mask generated by warping mask dataset is a single channel double
            // matrix
            // detailed mask -> watertight bit supported
            fillMetatile<double>
                (metatile, *src, origin, end, MetaFlags::watertight);
        } else {
            // mask layer from warped dataset is a single channel std::uint8_t
            // matrix
            // simple mask -> watertight bit not supported
            fillMetatile<std::uint8_t>
                (metatile, *src, origin, end, MetaFlags::available);
        }
    }

    // serialize metatile
    std::vector<unsigned char> buf;
    // write as png file
    cv::imencode(".png", metatile, buf
                 , { cv::IMWRITE_PNG_COMPRESSION, 9 });

    // reset max age received from dataset if mask is used
    if (maskDataset_) { ds.maxAge = boost::none; }
    sink.content(buf, fi.sinkFileInfo().setMaxAge(ds.maxAge));
}

} // namespace generator
