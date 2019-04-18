/**
 * Copyright (c) 2018 Melown Technologies SE
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

#ifndef mapproxy_definition_geodata_hpp_included_
#define mapproxy_definition_geodata_hpp_included_

#include <boost/optional.hpp>
#include <boost/filesystem.hpp>

#include "geo/geodataset.hpp"
#include "geo/heightcoding.hpp"

#include "../support/layerenancer.hpp"
#include "../support/geo.hpp"
#include "../resource.hpp"

#include "../heightfunction.hpp"

namespace resource {

// geodata vector formats

struct GeodataIntrospection {
    boost::optional<Resource::Id> surface;
    boost::any browserOptions;

    bool empty() const;
    bool operator!=(const GeodataIntrospection &other) const;

    void parse(const Json::Value &value);
    void build(Json::Value &value) const;
};

struct GeodataVectorBase : public DefinitionBase {
    typedef GeodataIntrospection Introspection;

    /** Input dataset (can be remote url, interpreted as a template by tiled
     *  version.
     */
    std::string dataset;
    DemDataset dem;
    boost::optional<geo::heightcoding::Config::LayerNames> layers;
    boost::optional<geo::heightcoding::Config::LayerNames> clipLayers;
    geo::VectorFormat format;
    geo::vectorformat::Config formatConfig;
    std::string styleUrl;
    int displaySize;
    geo::heightcoding::Mode mode;
    LayerEnhancer::map layerEnhancers;
    HeightFunction::pointer heightFunction;
    boost::any options;

    Introspection introspection;

    GeodataVectorBase()
        : format(geo::VectorFormat::geodataJson) , displaySize(256)
        , mode(geo::heightcoding::Mode::auto_)
    {}

    virtual void from_impl(const Json::Value &value);
    virtual void to_impl(Json::Value &value) const;

protected:
    virtual Changed changed_impl(const DefinitionBase &other) const;
};

struct GeodataVector : public GeodataVectorBase {
public:
    GeodataVector() {}

    static constexpr Resource::Generator::Type type
        = Resource::Generator::Type::geodata;
    static constexpr char driverName[] = "geodata-vector";

private:
    virtual bool frozenCredits_impl() const { return false; }

    virtual bool needsRanges_impl() const { return false; }
};

struct GeodataVectorTiled : GeodataVectorBase {
    /** Maximum available LOD in the source data. Detailed LODs will be
     *  generated from coarser tiles at maxSourceLod.
     *  LOD is in local subtree.
     */
    boost::optional<vts::Lod> maxSourceLod;

    virtual void from_impl(const Json::Value &value);
    virtual void to_impl(Json::Value &value) const;

    GeodataVectorTiled() {}

    static constexpr Resource::Generator::Type type
        = Resource::Generator::Type::geodata;
    static constexpr char driverName[] = "geodata-vector-tiled";

private:
    virtual Changed changed_impl(const DefinitionBase &other) const;
};

/** Mesh to monolithic geodata generator.
 */
struct GeodataMesh : public DefinitionBase {
    typedef GeodataIntrospection Introspection;

    static constexpr Resource::Generator::Type type
        = Resource::Generator::Type::geodata;
    static constexpr char driverName[] = "geodata-mesh";

    /** Path to mesh file (OBJ or PLY).
     */
    std::string dataset;

    /** Mesh SRS.
     */
    geo::SrsDefinition srs;

    /** Is SRS vertically adjusted?
     */
    bool adjustVertical;

    /** Mesh center in mesh SRS. Defaults to [0,0,0].
     */
    math::Point3 center;

    geo::VectorFormat format;
    geo::vectorformat::Config formatConfig;
    std::string styleUrl;
    int displaySize;
    boost::any options;

    Introspection introspection;

    GeodataMesh()
        : adjustVertical(false), format(geo::VectorFormat::geodataJson)
        , displaySize(256)
    {}

    virtual void from_impl(const Json::Value &value);
    virtual void to_impl(Json::Value &value) const;

protected:
    virtual Changed changed_impl(const DefinitionBase &other) const;
    virtual bool needsRanges_impl() const { return false; }
};

// helper functions

inline bool differ(const geo::vectorformat::GeodataConfig &l
                   , const geo::vectorformat::GeodataConfig *r) {
    // different type?
    if (!r) { return true; }
    // same types
    if (l.resolution != r->resolution) { return true; }
    return false;
}

inline bool differ(const geo::vectorformat::Config &l
                   , const geo::vectorformat::Config &r)
{
    if (const auto *cl = boost::get<geo::vectorformat::GeodataConfig>(&l)) {
        return differ(*cl, boost::get<geo::vectorformat::GeodataConfig>(&r));
    }
    // add more config types here
    return true;
}

} // namespace resource

#endif // mapproxy_definition_geodata_hpp_included_
