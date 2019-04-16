/**
 * Copyright (c) 2019 Melown Technologies SE
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

#ifndef mapproxy_generator_tms_raster_base_hpp_included_
#define mapproxy_generator_tms_raster_base_hpp_included_

#include <boost/optional.hpp>

#include "vts-libs/registry/extensions.hpp"
#include "vts-libs/storage/support.hpp"

#include "../support/wmts.hpp"
#include "../generator.hpp"

namespace vre = vtslibs::registry::extensions;
namespace vs = vtslibs::storage;

namespace generator {

class TmsRasterBase : public Generator {
public:
    TmsRasterBase(const Params &params
                  , const boost::optional<RasterFormat> &format
                  = boost::none);

private:
    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , Sink &sink) const;

    virtual Task generateVtsFile_impl(const FileInfo &fileInfo
                                      , Sink &sink) const = 0;

    Task wmtsInterface(const FileInfo &fileInfo, Sink &sink) const;

    wmts::WmtsResources wmtsResources(const WmtsFileInfo &fileInfo) const;

    std::string wmtsReadme() const;

    const vre::Wmts& getWmts() const;

    RasterFormat format_;
    const vre::Wmts *wmts_;
};

} // namespace generator

#endif // mapproxy_generator_tms_raster_base_hpp_included_
