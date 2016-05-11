#ifndef geo_ogrsupport_hpp_included_
#define geo_ogrsupport_hpp_included_

#include <memory>

#include <boost/filesystem/path.hpp>

#include <ogr_api.h>
#include <ogrsf_frmts.h>

namespace geo {

typedef std::shared_ptr< ::GDALDataset> VectorDataset;
typedef std::shared_ptr< ::OGRFeature> Feature;
typedef std::shared_ptr< ::OGRGeometry> Geometry;
typedef std::vector<Geometry> Geometries;

VectorDataset openVectorDataset(const boost::filesystem::path &path);
VectorDataset createVectorDataset(const boost::filesystem::path &path
                                  , ::GDALDriver *driver);
VectorDataset createVectorDataset(const boost::filesystem::path &path
                                  , VectorDataset &driverSource);
Feature feature(::OGRFeatureH f);
Geometry geometry(::OGRGeometryH g, bool clone = false);

} // namespace geo

#endif // geo_ogrsupport_hpp_included_
