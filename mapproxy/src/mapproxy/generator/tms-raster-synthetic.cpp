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

#include "geo/geodataset.hpp"

#include "imgproc/rastermask/cvmat.hpp"
#include "imgproc/png.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/qtree-rasterize.hpp"
#include "vts-libs/vts/opencv/colors.hpp"

#include "../error.hpp"
#include "../support/metatile.hpp"
#include "../support/tileindex.hpp"
#include "../support/revision.hpp"

#include "tms-raster-synthetic.hpp"
#include "factory.hpp"
#include "../support/python.hpp"

#include "browser2d/index.html.hpp"

namespace fs = boost::filesystem;
namespace bgil = boost::gil;
namespace vr = vtslibs::registry;

namespace generator {

detail::TmsRasterSyntheticMFB
::TmsRasterSyntheticMFB(const Generator::Params &params)
    : definition_(params.resource.definition<Definition>())
{}

TmsRasterSynthetic::TmsRasterSynthetic(const Params &params)
    : detail::TmsRasterSyntheticMFB(params)
    , TmsRasterBase(params, definition_.format)
    , hasMetatiles_(false)
{
    // not seen or index-less
    LOG(info1) << "Generator for <" << id() << "> not ready.";
}

void TmsRasterSynthetic::prepare_impl(Arsenal&)
{
    LOG(info2) << "Preparing <" << id() << ">.";

    // try to open datasets
    if (definition_.mask) {
        geo::GeoDataset::open(absoluteDataset(*definition_.mask));
        // we have mask dataset -> metatiles exist
        hasMetatiles_ = true;
    }

    makeReady();
}

vr::BoundLayer TmsRasterSynthetic::boundLayer(ResourceRoot root) const
{
    const auto &res(resource());

    vr::BoundLayer bl;
    bl.id = res.id.fullId();
    bl.numericId = 0; // no numeric ID
    bl.type = vr::BoundLayer::Type::raster;

    // build url
    bl.url = prependRoot
        (utility::format("{lod}-{x}-{y}.%s%s"
                         , definition_.format
                         , RevisionWrapper(res.revision, "?"))
         , resource(), root);
    if (definition_.mask) {
        bl.maskUrl = prependRoot
            (utility::format("{lod}-{x}-{y}.mask%s"
                             , RevisionWrapper(res.revision, "?"))
             , resource(), root);
        if (hasMetatiles_) {
            const auto fname
                (utility::format("{lod}-{x}-{y}.meta%s"
                                 , RevisionWrapper(res.revision, "?")));

            bl.metaUrl = prependRoot(fname, resource(), root);
        }
    }

    bl.lodRange = res.lodRange;
    bl.tileRange = res.tileRange;
    bl.credits = asInlineCredits(res);

    bl.options = definition_.options;

    // done
    return bl;
}

vts::MapConfig TmsRasterSynthetic::mapConfig_impl(ResourceRoot root)
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

Generator::Task TmsRasterSynthetic
::generateVtsFile_impl(const FileInfo &fileInfo, Sink &sink) const
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

    case TmsFileInfo::Type::image:
        return [=](Sink &sink, Arsenal &arsenal) {
            generateTileImage(fi.tileId, fi.sinkFileInfo(), fi.format
                              , sink, arsenal);
        };

    case TmsFileInfo::Type::mask:
        return [=](Sink &sink, Arsenal &arsenal)  {
            generateTileMask(fi.tileId, fi, sink, arsenal);
        };

    case TmsFileInfo::Type::metatile:
        return [=](Sink &sink, Arsenal &arsenal) {
            generateMetatile(fi.tileId, fi, sink, arsenal);
        };
    }

    return {};
}

void TmsRasterSynthetic::generateTileImage(const vts::TileId &tileId
                                           , Sink::FileInfo &&sfi
                                           , RasterFormat format
                                           , Sink &sink, Arsenal&
                                           , bool dontOptimize) const
{
    sink.checkAborted();
    if (format != definition_.format) {
        return sink.error
            (utility::makeError<NotFound>
             ("Format <%s> is not supported by this resource (%s)."
              , format, definition_.format));
    }

    const vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        return sink.error
            (utility::makeError<NotFound>
             ("TileId outside of valid reference frame tree."));
    }

    bool valid(true);
    if (!nodeInfo.productive()) {
        if (!dontOptimize) {
            return sink.error
                (utility::makeError<EmptyImage>("No valid data."));
        }

        // invalid but we cannot optimize
        valid = false;
    }

    // generate image
    const auto tile(valid
                    ? generateTileImage(tileId)
                    : cv::Mat_<cv::Vec3b>(vr::BoundLayer::tileHeight
                                          , vr::BoundLayer::tileWidth
                                          , cv::Vec3b(0, 0, 0)));

    // serialize
    std::vector<unsigned char> buf;
    switch (format) {
    case RasterFormat::jpg:
        // TODO: configurable quality
        cv::imencode(".jpg", tile, buf
                     , { cv::IMWRITE_JPEG_QUALITY, 75 });
        break;

    case RasterFormat::png:
        cv::imencode(".png", tile, buf
                     , { cv::IMWRITE_PNG_COMPRESSION, 9 });
        break;
    }

    sink.content(buf, sfi);
}

void TmsRasterSynthetic::generateTileMask(const vts::TileId &tileId
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

    if (!nodeInfo.productive()) {
        sink.error(utility::makeError<EmptyImage>("No valid data."));
        return;
    }

    if (!definition_.mask) {
        cv::Mat_<std::uint8_t> mask(vr::BoundLayer::tileHeight
                                    , vr::BoundLayer::tileWidth, 255);

        // serialize
        std::vector<unsigned char> buf;
        // write as png file
        cv::imencode(".png", mask, buf
                     , { cv::IMWRITE_PNG_COMPRESSION, 9 });

        // reset max age received from dataset if mask is uded
        sink.content(buf, fi.sinkFileInfo());
        return;
    }

    auto mask(arsenal.warper.warp
              (GdalWarper::RasterRequest
               (GdalWarper::RasterRequest::Operation::mask
                , absoluteDataset(*definition_.mask)
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

    // reset max age received from dataset if mask is uded
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

void meta2d(const vts::TileIndex &tileIndex, const vts::TileId &tileId
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

} // namespace

void TmsRasterSynthetic::generateMetatile(const vts::TileId &tileId
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

    // non-tileindex code
    cv::Mat metatile(Constants::RasterMetatileSize.width
                     , Constants::RasterMetatileSize.height
                     , CV_8U, cv::Scalar(0));

    // bits to set for watertight tile
    const auto watertightBits
        (definition_.mask
         ? MetaFlags::watertight // detailed mask -> watertight supported
         : MetaFlags::available // no mask -> watertight supported
         );

    for (const auto &block : blocks) {
        if (!block.commonAncestor.productive()) { continue; }

        const auto &view(block.view);
        math::Size2 bSize(vts::tileRangesSize(view));

        GdalWarper::Raster src;
        // warp detailed mask
        src = arsenal.warper.warp
            (GdalWarper::RasterRequest
             (GdalWarper::RasterRequest::Operation::detailMask
              , absoluteDataset(*definition_.mask)
              , vr::system.srs(block.srs).srsDef
              , block.extents, bSize)
             , sink);
        sink.checkAborted();

        // generate metatile content for current block
        math::Point2i origin(view.ll(0) - tileId.x, view.ll(1) - tileId.y);
        math::Point2i end(view.ur(0) - tileId.x, view.ur(1) - tileId.y);
        for (int j(origin(1)), je(end(1)), jj(0); j <= je; ++j, ++jj) {
            for (int i(origin(0)), ie(end(0)), ii(0); i <= ie; ++i, ++ii) {
                const auto &in(src->at<double>(jj, ii));
                auto &out(metatile.at<std::uint8_t>(j, i));
                if (!in) {
                    // black -> not available
                    out = MetaFlags::unavailable;
                } else if (in >= 255) {
                    // white -> available and watertight
                    // -> use computed bits above
                    out = watertightBits;
                } else {
                    // gray -> available but not watetight
                    out = MetaFlags::available;
                }
            }
        }
    }

    // serialize metatile
    std::vector<unsigned char> buf;
    // write as png file
    cv::imencode(".png", metatile, buf
                 , { cv::IMWRITE_PNG_COMPRESSION, 9 });

    // reset max age received from dataset if mask is uded
    sink.content(buf, fi.sinkFileInfo());
}

} // namespace generator
