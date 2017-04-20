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

#ifndef mapproxy_generator_geodata_vector_hpp_included_
#define mapproxy_generator_geodata_vector_hpp_included_

#include "vts-libs/vts/urltemplate.hpp"
#include "vts-libs/vts/tileset/tilesetindex.hpp"

#include "./geodatavectorbase.hpp"

namespace generator {

class GeodataVector : public GeodataVectorBase {
public:
    GeodataVector(const Params &params);

    struct Definition : public GeodataVectorBase::Definition {
    public:
        Definition() : GeodataVectorBase::Definition() {}
    private:
        virtual bool frozenCredits_impl() const { return false; }
    };

private:
    virtual void prepare_impl(Arsenal &arsenal);

    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const;
    virtual vr::FreeLayer freeLayer_impl(ResourceRoot root) const;

    virtual void generateMetatile(Sink &sink
                                  , const GeodataFileInfo &fileInfo
                                  , Arsenal &arsenal) const;

    virtual void generateGeodata(Sink &sink
                                 , const GeodataFileInfo &fileInfo
                                 , Arsenal &arsenal) const;

    GdalWarper::Heightcoded::pointer
    heightcode(const DemDataset::list &datasets
               , GdalWarper &warper, Aborter &aborter) const;

    Definition definition_;

    const DemDataset dem_;

    const vr::Srs physicalSrs_;

    /** Output metadata.
     */
    geo::heightcoding::Metadata metadata_;

    /** Path to cached output data.
     */
    boost::filesystem::path dataPath_;
};

} // namespace generator

#endif // mapproxy_generator_geodata_vector_hpp_included_
