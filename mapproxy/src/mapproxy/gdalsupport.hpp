#ifndef mapproxy_gdalsupport_hpp_included_
#define mapproxy_gdalsupport_hpp_included_

#include <memory>

#include <boost/optional.hpp>

#include <opencv2/core/core.hpp>

#include "geo/srsdef.hpp"
#include "geo/geodataset.hpp"

class GdalWarper {
public:
    typedef std::shared_ptr<GdalWarper> pointer;

    GdalWarper(unsigned int processCount);

    typedef std::shared_ptr<cv::Mat> Raster;

    class RasterRequest {
    public:
        enum class Operation { image, mask };

        Operation operation;
        std::string dataset;
        boost::optional<std::string> mask;
        geo::SrsDefinition srs;
        math::Extents2 extents;
        math::Size2 size;
        geo::GeoDataset::Resampling resampling;

        RasterRequest(Operation operation
                      , const std::string &dataset
                      , const boost::optional<std::string> &mask
                      , const geo::SrsDefinition &srs
                      , const math::Extents2 &extents
                      , const math::Size2 &size
                      , geo::GeoDataset::Resampling resampling)
            : operation(operation), dataset(dataset), mask(mask)
            , srs(srs), extents(extents), size(size), resampling(resampling)
        {}
    };

    Raster warp(const RasterRequest &request);

    struct Detail;

private:
    std::shared_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};

#endif // mapproxy_gdalsupport_hpp_included_
