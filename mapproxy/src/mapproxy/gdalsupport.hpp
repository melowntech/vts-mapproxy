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

#ifndef mapproxy_gdalsupport_hpp_included_
#define mapproxy_gdalsupport_hpp_included_

#include <memory>
#include <chrono>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>

#include "utility/runnable.hpp"

#include "geo/srsdef.hpp"
#include "geo/geodataset.hpp"
#include "geo/vectorformat.hpp"
#include "geo/heightcoding.hpp"

#include "vts-libs/vts/opencv/navtile.hpp"
#include "vts-libs/vts/nodeinfo.hpp"

#include "./support/geo.hpp"
#include "./sink.hpp"

namespace vts = vtslibs::vts;

class GdalWarper {
public:
    typedef std::shared_ptr<GdalWarper> pointer;

    struct Options {
        unsigned int processCount;
        boost::filesystem::path tmpRoot;
        std::size_t rssCheckPeriod;
        std::size_t rssLimit;

        Options()
            : processCount(5), rssCheckPeriod(5)
            , rssLimit(std::size_t(1) << 12)
        {}
    };

    GdalWarper(const Options &options, utility::Runnable &runnable);

    typedef std::shared_ptr<cv::Mat> Raster;

    class RasterRequest {
    public:
        /** Operations:
         *  * image:
         *        warps image from dataset, uses optional mask
         *        returns N channel 8bit image
         *  * mask:
         *  * maskNoOpt:
         *        warps dataset and return its mask
         *        returns single channel 8bit mask
         *        mask: throws FullImage exception when all pixels are valid
         *
         *  * detailMask:
         *        warps dataset with agerage filter and no nodata values
         *        returns grayscale where value > 0 and < 255 marks partially
         *        covered pixels
         *
         * * dem:
         *       warps dataset as a DEM using Resampling::dem filter
         *       returns single channel double matrix
         *
         *       warp is done in grid registration (i.e. provided extents are
         *       inflated by half pixel in each direction and raster size is
         *       incremented by one.
         *
         * * demOptimal:
         *       warps dataset as a DEM using Resampling::dem filter
         *       returns single channel double matrix
         *
         *       result raster size is computed from scaling factor, provided
         *       size is upper limit (lower limit is 2x2)
         *
         *       warp is done in grid registration (i.e. provided extents are
         *       inflated by half pixel in each direction and raster size is
         *       incremented by one.
         *
         * * valueMinMax:
         *       warps dataset using given filter, dataset.min by minimum filter
         *       and dataset.max by maximum filter
         *
         *       returns 3-channel double matrix with current value, minimum
         *       value and maximum value in each pixel
         */
        enum class Operation {
            image, mask, maskNoOpt, detailMask, dem, demOptimal, valueMinMax
        };

        Operation operation;
        std::string dataset;
        geo::SrsDefinition srs;
        math::Extents2 extents;
        math::Size2 size;
        geo::GeoDataset::Resampling resampling;
        boost::optional<std::string> mask;

        RasterRequest(Operation operation
                      , const std::string &dataset
                      , const geo::SrsDefinition &srs
                      , const math::Extents2 &extents
                      , const math::Size2 &size
                      , geo::GeoDataset::Resampling resampling
                      = geo::GeoDataset::Resampling::nearest
                      , const boost::optional<std::string> &mask
                      = boost::none)
            : operation(operation), dataset(dataset)
            , srs(srs), extents(extents), size(size), resampling(resampling)
            , mask(mask)
        {}
    };

    Raster warp(const RasterRequest &request, Aborter &sink);

    struct Heightcoded {
        typedef std::shared_ptr<Heightcoded> pointer;

        const char *data;
        std::size_t size;

        geo::heightcoding::Metadata metadata;

        Heightcoded(const char *data, std::size_t size
                   , const geo::heightcoding::Metadata &metadata)
            : data(data), size(size), metadata(metadata)
        {}
    };

    // Vector operations

    /** Heightcode vector ds using raster ds
     */
    Heightcoded::pointer
    heightcode(const std::string &vectorDs
               , const DemDataset::list &rasterDs
               , const geo::heightcoding::Config &config
               , const boost::optional<std::string> &vectorGeoidGrid
               , Aborter &aborter);

    /** Navigation tile info.
     */
    struct Navtile {
        std::string path;
        std::string raw;
        math::Extents2 extents;
        std::string sdsSrs;
        std::string navSrs;
        vts::NavTile::HeightRange heightRange;

        Navtile() {}
    };

    /** Heightcode vector ds using navtile ds
     */
    Heightcoded::pointer
    heightcode(const std::string &vectorDs
               , const Navtile &navtile
               , const geo::heightcoding::Config &config
               , const std::string &fallbackDs
               , const boost::optional<std::string> &geoidGrid
               , Aborter &aborter);

    /** Do housekeeping. Must be called in the process where internals are being
     * run.
     */
    void housekeeping();

    struct Detail;

private:
    std::shared_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};

#endif // mapproxy_gdalsupport_hpp_included_
