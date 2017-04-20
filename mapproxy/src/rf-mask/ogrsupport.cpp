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
