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

#ifndef mapproxy_generator_geodata_semantic_hpp_included_
#define mapproxy_generator_geodata_semantic_hpp_included_

#include "../support/mmapped/tilesetindex.hpp"

#include "../heightfunction.hpp"
#include "../generator.hpp"
#include "../definition.hpp"

namespace generator {

class GeodataSemanticTiled : public Generator {
public:
    GeodataSemanticTiled(const Params &params);

    typedef resource::GeodataSemanticTiled Definition;

private:
    virtual void prepare_impl(Arsenal &arsenal);

    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , Sink &sink) const;

    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const;
    vr::FreeLayer freeLayer(ResourceRoot root) const;

    void generateGeodata(Sink &sink, const GeodataFileInfo &fileInfo
                         , Arsenal &arsenal) const;

    void generateMetatile(Sink &sink, const GeodataFileInfo &fi
                          , Arsenal &arsenal) const;

    Definition definition_;

    /** Path to /dem dataset
     */
    const DemDataset dem_;

    /** URL to style.
     */
    std::string styleUrl_;

    /** Path to style file if definition.styleUrl starts with `file:`.
     */
    boost::filesystem::path stylePath_;

    /** Dataset path.
     */
    const boost::filesystem::path dataset_;

    const vr::Srs &physicalSrs_;

    geo::vectorformat::GeodataConfig geodataConfig_;

    boost::optional<mmapped::Index> index_;
};

} // namespace generator

#endif // mapproxy_generator_geodata_semantic_hpp_included_
