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
#include "vts-libs/vts/qtree-rasterize.hpp"
#include "vts-libs/vts/opencv/colors.hpp"

#include "../error.hpp"
#include "../support/metatile.hpp"
#include "../support/tileindex.hpp"
#include "../support/revision.hpp"

#include "./tms-raster-patchwork.hpp"
#include "./factory.hpp"
#include "../support/python.hpp"

#include "browser2d/index.html.hpp"

namespace fs = boost::filesystem;
namespace bgil = boost::gil;
namespace vr = vtslibs::registry;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<TmsRasterPatchwork>(params);
    }

    virtual bool systemInstance() const { return true; }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType<TmsRasterPatchwork>(std::make_shared<Factory>());
});

} // namespace

TmsRasterPatchwork::TmsRasterPatchwork(const Params &params)
    : TmsRasterSynthetic(params)
    , definition_(resource().definition<Definition>())
{}

void TmsRasterPatchwork::generateTileImage(const vts::TileId &tileId
                                           , const TmsFileInfo &fi
                                           , const vts::NodeInfo&
                                           , Sink &sink
                                           , Arsenal&) const
{
    unsigned long long int colorIndex(tileId.y);
    colorIndex <<= tileId.lod;
    colorIndex += tileId.x;
    // skip black
    colorIndex = 1 + (colorIndex % 254);

    cv::Mat_<cv::Vec3b> tile(vr::BoundLayer::tileHeight
                             , vr::BoundLayer::tileWidth);

    const cv::Vec3b color(vts::opencv::palette256vec[colorIndex]);
    const cv::Vec3b darkColor(color[0] * 0.8, color[1] * 0.8, color[2] * 0.8);
    cv::Vec3b colors[2] = { color, darkColor };

    for (int j(0); j < vr::BoundLayer::tileHeight; ++j) {
        for (int i(0); i < vr::BoundLayer::tileWidth; ++i) {
            tile(j, i) = colors[((j >> 3) + (i >> 3)) & 1];
        }
    }

    // serialize
    std::vector<unsigned char> buf;
    switch (fi.format) {
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

    sink.content(buf, fi.sinkFileInfo());
}

} // namespace generator
