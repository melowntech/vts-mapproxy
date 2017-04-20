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
