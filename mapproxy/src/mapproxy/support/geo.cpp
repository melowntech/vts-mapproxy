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

#include "vts-libs/vts/csconvertor.hpp"

#include "geo.hpp"

namespace ublas = boost::numeric::ublas;

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

::OGRSpatialReference localTm(const vr::ReferenceFrame &rf
                              , const geo::SrsDefinition &srsDef
                              , const math::Point2d point)
{
    auto navSrs(vr::system.srs(rf.model.navigationSrs).srsDef.reference());

    ::OGRSpatialReference latlon;
    if (OGRERR_NONE != latlon.CopyGeogCSFrom(&navSrs)) {
        LOGTHROW(err3, std::runtime_error)
            << "Cannot copy GeoCS from navigation SRS.";
    }

    const auto llCenter(geo::CsConvertor(srsDef, latlon)(point));

    // construct tmerc
    ::OGRSpatialReference tm;
    if (OGRERR_NONE != tm.CopyGeogCSFrom(&navSrs)) {
        LOGTHROW(err3, std::runtime_error)
            << "Cannot copy GeoCS from navigation SRS.";
    }
    if (OGRERR_NONE != tm.SetTM(llCenter(1), llCenter(0), 1.0, 0.0, 0.0)) {
        LOGTHROW(err3, std::runtime_error)
            << "Cannot set tmerc.";
    }

    return tm;
}

math::Matrix4 makePlaneTrafo(const vr::ReferenceFrame &rf
                             , const math::Point2 &navCenter)
{
    // construct convertor from local tmerc to physical system
    const auto navSrs(vr::system.srs(rf.model.navigationSrs).srsDef);
    const vts::CsConvertor tm2phys(localTm(rf, navSrs, navCenter)
                                   , rf.model.physicalSrs);

    const auto center(tm2phys(math::Point3d(0, 0, 0)));
    const auto eye(tm2phys(math::Point3d(0, 0, -1)));
    const auto upOfCenter(tm2phys(math::Point3d(0, 1, 0)));

    const auto look(math::Point3(eye - center));
    const auto up(math::Point3(upOfCenter - center));

#if 0
    LOG(info4) << std::fixed << "center: " << center;
    LOG(info4) << std::fixed << "eye: " << eye;
    LOG(info4) << std::fixed << "upOfCenter: " << upOfCenter;
    LOG(info4) << std::fixed << "look: " << look;
    LOG(info4) << std::fixed << "up: " << up;
#endif

    // ec to wc camera
    math::Matrix4 ec2wc(4, 4);

    // fetch all 4 columns of EC2WC matrix
    auto e1_(ublas::column(ec2wc, 0));
    auto e2_(ublas::column(ec2wc, 1));
    auto e3_(ublas::column(ec2wc, 2));
    auto e4_(ublas::column(ec2wc, 3));

    // fetch only first 3 elements of each column of EC2WC matrix
    // (cannot be chained because resulting vector expression is read-only :()
    auto e1(ublas::subrange(e1_, 0, 3));
    auto e2(ublas::subrange(e2_, 0, 3));
    auto e3(ublas::subrange(e3_, 0, 3));
    auto e4(ublas::subrange(e4_, 0, 3));

    // reset last row to (0, 0, 0, 1)
    ublas::row(ec2wc, 3) = ublas::unit_vector<double>(4, 3);

    e3 = math::normalize(math::normalize(look));
    e1 = math::normalize(math::crossProduct(up, e3));
    e2 = math::crossProduct(e3, e1);
    e4 = eye;

    return math::matrixInvert(ec2wc);
}
