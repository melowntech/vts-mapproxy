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

    struct Raster {
        cv::Mat mat;
        std::shared_ptr<char*> memory;
    };

    Raster warpImage(const std::string &dataset
                     , const boost::optional<std::string> &mask
                     , const geo::SrsDefinition &srs
                     , const math::Extents2 &extents
                     , const math::Size2 &size
                     , geo::GeoDataset::Resampling resampling);

    Raster warpMask(const std::string &dataset
                    , const boost::optional<std::string> &mask
                    , const geo::SrsDefinition &srs
                    , const math::Extents2 &extents
                    , const math::Size2 &size
                    , geo::GeoDataset::Resampling resampling);

    struct Detail;

private:
    std::shared_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};

#endif // mapproxy_gdalsupport_hpp_included_
