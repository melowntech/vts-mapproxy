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

#include "math/math.hpp"

#include "geo/csconvertor.hpp"

#include "./geo.hpp"

double tileCircumference(const math::Extents2 &extents
                         , const geo::SrsDefinition &srs
                         , const geo::GeoDataset &dataset
                         , int samples)
{
    geo::CsConvertor conv(srs, dataset.srs());

    auto es(math::size(extents));
    math::Size2f step(es.width / samples, es.height / samples);

    double length(0.0);

    try {
        // previous point, i.e. first
        auto prev(dataset.geo2raster<math::Point2>(conv(extents.ll)));

        auto add([&](const math::Point2 &p)
        {
            auto px(dataset.geo2raster<math::Point2>(conv(p)));
            length += boost::numeric::ublas::norm_2(px - prev);
            prev = px;
        });

        // bottom side
        for (int i(1); i <= samples; ++i) {
            add(math::Point2(extents.ll(0) + i * step.width, extents.ll(1)));
        }

        // right side
        for (int i(1); i <= samples; ++i) {
            add(math::Point2(extents.ur(0), extents.ll(1) + i * step.height));
        }

        // top side
        for (int i(1); i <= samples; ++i) {
            add(math::Point2(extents.ur(0) - i * step.width, extents.ur(1)));
        }

        // left side
        for (int i(1); i <= samples; ++i) {
            add(math::Point2(extents.ll(0), extents.ur(1) - i * step.height));
        }
    } catch (...) {
        return std::numeric_limits<double>::infinity();
    }

    return length;
}


math::Extents2 extentsPlusHalfPixel(const math::Extents2 &extents
                                    , const math::Size2 &pixels)
{
    auto es(math::size(extents));
    const math::Size2f px(es.width / pixels.width, es.height / pixels.height);
    const math::Point2 hpx(px.width / 2, px.height / 2);
    return math::Extents2(extents.ll - hpx, extents.ur + hpx);
}
