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

#ifndef mapproxy_generator_surface_meta_hpp_included_
#define mapproxy_generator_surface_meta_hpp_included_

#include "../generator.hpp"
#include "../definition.hpp"

#include "providers.hpp"

namespace vts = vtslibs::vts;
namespace vr = vtslibs::registry;

namespace generator {

class SurfaceMeta : public Generator {
public:
    SurfaceMeta(const Params &params);

    ~SurfaceMeta();

    typedef resource::SurfaceMeta Definition;

private:
    virtual void prepare_impl(Arsenal &arsenal);
    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const ;

    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , Sink &sink) const;

    Generator::Task tileindex(const SurfaceFileInfo &fileInfo, Sink &sink)
        const;

    vts::FullTileSetProperties properties() const;

    const Definition &definition_;

    pointer surface_;
    pointer tms_;

    /** Pointer owned by surface.
     */
    VtsTilesetProvider *ts_;

    /** Pointer owned by tms.
     */
    VtsAtlasProvider *atlas_;

    MetatileOverrides metatileOverrides_;
};

} // namespace generator

#endif // mapproxy_generator_surface_meta_hpp_included_
