#ifndef mapproxy_gdalsupport_requests_hpp_included_
#define mapproxy_gdalsupport_requests_hpp_included_

#include "../gdalsupport.hpp"
#include "./types.hpp"

class ShRequestBase {
public:
    virtual ~ShRequestBase() {}

    void done() { done_impl(); }

private:
    virtual void done_impl() = 0;
};

class ShRaster : boost::noncopyable {
public:
    ShRaster(const GdalWarper::RasterRequest &other
             , ManagedBuffer &sm, ShRequestBase *owner);

    ~ShRaster();

    operator GdalWarper::RasterRequest() const;

    /** Steals response.
     */
    cv::Mat* response();
    void response(bi::interprocess_mutex &mutex, cv::Mat *response);

private:
    ManagedBuffer &sm_;
    ShRequestBase *owner_;

    GdalWarper::RasterRequest::Operation operation_;
    String dataset_;
    String srs_;
    geo::SrsDefinition::Type srsType_;
    math::Extents2 extents_;
    math::Size2 size_;
    geo::GeoDataset::Resampling resampling_;
    String mask_;

    // response matrix
    cv::Mat *response_;
};

struct ShHeightCodeConfig {
    String workingSrs_;
    geo::SrsDefinition::Type workingSrsType_;

    String outputSrs_;
    geo::SrsDefinition::Type outputSrsType_;

    boost::optional<StringVector> layers_;

    boost::optional<math::Extents2> clipWorkingExtents_;

    geo::VectorFormat format_;

    ShHeightCodeConfig(const geo::heightcoding::Config &config
                       , ManagedBuffer &sm);

    operator geo::heightcoding::Config() const;
};

class ShHeightCode : boost::noncopyable {
public:
    ShHeightCode(const std::string &vectorDs, const std::string &rasterDs
                 , const geo::heightcoding::Config &config
                 , const boost::optional<std::string> &geoidGrid
                 , ManagedBuffer &sm, ShRequestBase *owner);

    ~ShHeightCode();

    std::string vectorDs() const;

    std::string rasterDs() const;

    geo::heightcoding::Config config() const;

    boost::optional<std::string> geoidGrid() const;

    /** Steals response.
     */
    GdalWarper::Heighcoded* response();

    void response(bi::interprocess_mutex &mutex
                  , GdalWarper::Heighcoded *response);

private:
    ManagedBuffer &sm_;
    ShRequestBase *owner_;
    String vectorDs_;
    String rasterDs_;
    ShHeightCodeConfig config_;
    String geoidGrid_;

    // response memory block
    GdalWarper::Heighcoded *response_;
};

struct ShNavtile {
    String path;
    String raw;
    math::Extents2 extents;
    String sdsSrs;
    String navSrs;
    vts::NavTile::HeightRange heightRange;

    ShNavtile(const GdalWarper::Navtile &navtile, ManagedBuffer &sm);

    GdalWarper::Navtile navtile(bool noRaw = false) const;

    ConstBlock rawData() const;
};

class ShNavHeightCode : boost::noncopyable {
public:
    ShNavHeightCode(const std::string &vectorDs
                    , const GdalWarper::Navtile &navtile
                    , const geo::heightcoding::Config &config
                    , ManagedBuffer &sm, ShRequestBase *owner);

    ~ShNavHeightCode();

    std::string vectorDs() const;

    GdalWarper::Navtile navtile(bool noRaw = false) const;

    geo::heightcoding::Config config() const;

    ConstBlock rawData() const;

    /** Steals response.
     */
    GdalWarper::Heighcoded* response();

    void response(bi::interprocess_mutex &mutex
                  , GdalWarper::Heighcoded *response);

private:
    ManagedBuffer &sm_;
    ShRequestBase *owner_;

    String vectorDs_;
    ShNavtile navtile_;
    ShHeightCodeConfig config_;

    // response memory block
    GdalWarper::Heighcoded *response_;
};

#endif // mapproxy_gdalsupport_requests_hpp_included_
