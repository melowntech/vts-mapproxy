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

#ifndef mapproxy_generator_tms_raster_synthetic_hpp_included_
#define mapproxy_generator_tms_raster_synthetic_hpp_included_

#include "vts-libs/vts/nodeinfo.hpp"

#include "../definition/tms.hpp"

#include "tms-raster-base.hpp"

namespace vts = vtslibs::vts;

// fwd
namespace cv { class Mat; }

namespace generator {

namespace detail {

/** Member from base idiom
 */
class TmsRasterSyntheticMFB {
protected:
    TmsRasterSyntheticMFB(const Generator::Params &params);
    typedef resource::TmsRasterSynthetic Definition;
    const Definition &definition_;
};

} // namespace detail

class TmsRasterSynthetic
    : private detail::TmsRasterSyntheticMFB
    , public TmsRasterBase
{
public:
    TmsRasterSynthetic(const Params &params);

    using TmsRasterSyntheticMFB::Definition;

private:
    virtual void prepare_impl(Arsenal &arsenal);
    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const;

    virtual Task generateVtsFile_impl(const FileInfo &fileInfo
                                      , Sink &sink) const;

    virtual void
    generateTileImage(const vts::TileId &tileId, Sink::FileInfo &&fi
                      , RasterFormat format, Sink &sink, Arsenal &arsenal
                      , bool dontOptimize = false) const;

    virtual cv::Mat generateTileImage(const vts::TileId &tileId) const = 0;

    void generateTileMask(const vts::TileId &tileId
                          , const TmsFileInfo &fi
                          , Sink &sink, Arsenal &arsenal) const;

    void generateMetatile(const vts::TileId &tileId
                          , const TmsFileInfo &fi
                          , Sink &sink, Arsenal &arsenal) const;

    vr::BoundLayer boundLayer(ResourceRoot root) const;

    bool hasMask() const;

    bool hasMetatiles_;
};

} // namespace generator

#endif // mapproxy_generator_tms_raster_synthetic_hpp_included_
