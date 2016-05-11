#include "dbglog/dbglog.hpp"

#include "./ogrsupport.hpp"

namespace geo {

VectorDataset openVectorDataset(const boost::filesystem::path &path)
{
    auto ds(::GDALOpenEx(path.c_str(), (GDAL_OF_VECTOR | GDAL_OF_READONLY)
                         , nullptr, nullptr, nullptr));

    if (!ds) {
        LOGTHROW(err2, std::runtime_error)
            << "Failed to open dataset " << path << ".";
    }

    return VectorDataset(static_cast< ::OGRDataSource*>(ds)
                         , [](::OGRDataSource *ds) { delete ds; });
}

VectorDataset createVectorDataset(const boost::filesystem::path &path
                                  , ::GDALDriver *driver)
{
    auto ds(driver->pfnCreateVectorOnly(driver, path.c_str(), nullptr));
    if (!ds) {
        LOGTHROW(err2, std::runtime_error)
            << "Failed to create output dataset " << path << ".";
    }
    return VectorDataset(ds);
}

VectorDataset createVectorDataset(const boost::filesystem::path &path
                                         , VectorDataset &driverSource)
{
    return createVectorDataset(path, driverSource->GetDriver());
}

Feature feature(::OGRFeatureH f)
{
    return Feature(static_cast< ::OGRFeature*>(f)
                   , [](::OGRFeature *f) {
                       if (f) OGR_F_Destroy(f); });
}

Geometry geometry(::OGRGeometryH g, bool clone)
{
    if (!g) { return {}; }
    if (clone) { g = OGR_G_Clone(g); }
    return Geometry(static_cast< ::OGRGeometry*>(g)
                    , [](::OGRGeometry *g)
                    { if (g) OGR_G_DestroyGeometry(g); });
}

} // namespace geo
