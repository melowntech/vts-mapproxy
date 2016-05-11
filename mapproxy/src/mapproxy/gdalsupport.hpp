#ifndef mapproxy_gdalsupport_hpp_included_
#define mapproxy_gdalsupport_hpp_included_

#include <memory>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>

#include "geo/srsdef.hpp"
#include "geo/geodataset.hpp"

#include "./contentgenerator.hpp"

class GdalWarper {
public:
    typedef std::shared_ptr<GdalWarper> pointer;

    struct Options {
        unsigned int processCount;
        boost::filesystem::path tmpRoot;

        Options() : processCount(5) {}
    };

    GdalWarper(const Options &options);

    typedef std::shared_ptr<cv::Mat> Raster;

    class RasterRequest {
    public:
        /** Operations:
         *  * image:
         *        warps image from dataset, uses optional mask
         *        returns N channel 8bit image
         *  * mask:
         *        warps dataset and return its mask
         *        returns single channel 8bit mask
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
         * * minMax:
         *       dataset.min by minimum filter
         *       and dataset.max by maximum filter
         *
         *       returns 2-channel double matrix with minimum value and maximum
         *       value in each pixel
         *
         * * valueMinMax:
         *       warps dataset using given filter, dataset.min by minimum filter
         *       and dataset.max by maximum filter

         *       returns 3-channel double matrix with current value, minimum
         *       value and maximum value in each pixel
         */
        enum class Operation {
            image, mask, detailMask, dem, demOptimal, minMax, valueMinMax
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

    Raster warp(const RasterRequest &request, const Sink::pointer &sink);

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
