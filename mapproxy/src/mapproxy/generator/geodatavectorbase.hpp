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

#ifndef mapproxy_generator_geodata_hpp_included_
#define mapproxy_generator_geodata_hpp_included_

#include <boost/optional.hpp>

#include "geo/heightcoding.hpp"

#include "../generator.hpp"

namespace generator {

class GeodataVectorBase : public Generator {
public:
    GeodataVectorBase(const Params &params, bool tiled);

    struct Introspection {
        boost::optional<Resource::Id> surface;
        boost::any browserOptions;

        bool empty() const;
        bool operator!=(const Introspection &other) const;
    };

    struct Definition : public DefinitionBase {
        /** Input dataset (can be remote url, interpreted as a template by tiled
         *  version.
         */
        std::string dataset;
        DemDataset dem;
        boost::optional<std::vector<std::string>> layers;
        geo::VectorFormat format;
        std::string styleUrl;
        int displaySize;
        geo::heightcoding::Mode mode;

        Introspection introspection;

        Definition()
            : format(geo::VectorFormat::geodataJson) , displaySize(256)
            , mode(geo::heightcoding::Mode::auto_)
        {}

        virtual void from_impl(const boost::any &value);
        virtual void to_impl(boost::any &value) const;

    private:
        virtual Changed changed_impl(const DefinitionBase &other) const;
    };

protected:
    /** Parses viewspec from HTTP query. Returns list of found datasets ending
     *  with fallback dataset. Second returned attribute is information whether
     *  all viewspec arguments have been found.
     */
    std::pair<DemDataset::list, bool>
    viewspec2datasets(const std::string &query, const DemDataset &fallback)
        const;

    const std::string& styleUrl() const { return styleUrl_; }

private:
    virtual vr::FreeLayer freeLayer_impl(ResourceRoot root) const = 0;

    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , Sink &sink) const;

    virtual void generateMetatile(Sink &sink
                                  , const GeodataFileInfo &fileInfo
                                  , Arsenal &arsenal) const = 0;

    virtual void generateGeodata(Sink &sink
                                 , const GeodataFileInfo &fileInfo
                                 , Arsenal &arsenal) const = 0;

private:
    const Definition &definition_;
    bool tiled_;

    /** URL to style.
     */
    std::string styleUrl_;

    /** Path to style file if definition.styleUrl starts with `file:`.
     */
    boost::filesystem::path stylePath_;
};

} // namespace generator

#endif // mapproxy_generator_geodata_hpp_included_
