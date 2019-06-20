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

#ifndef mapproxy_definition_geodata_semantic_hpp_included_
#define mapproxy_definition_geodata_semantic_hpp_included_

#include "geodata.hpp"

namespace vr = vtslibs::registry;

namespace resource {

struct GeodataSemanticBase : public DefinitionBase {
    typedef GeodataIntrospection Introspection;

    /** Path to semantic world file.
     */
    std::string dataset;

    geo::VectorFormat format;
    geo::vectorformat::Config formatConfig;
    std::string styleUrl;
    int displaySize;
    boost::any options;

    Introspection introspection;

    GeodataSemanticBase()
        : format(geo::VectorFormat::geodataJson), displaySize(256)
    {}

    virtual void from_impl(const Json::Value &value);
    virtual void to_impl(Json::Value &value) const;

protected:
    virtual Changed changed_impl(const DefinitionBase &other) const;
};

/** Semantic world description to monolithic geodata generator.
 */
struct GeodataSemantic : public GeodataSemanticBase {
    bool simplified = false;

    static constexpr Resource::Generator::Type type
        = Resource::Generator::Type::geodata;
    static constexpr char driverName[] = "geodata-semantic";

    GeodataSemantic() = default;

    virtual void from_impl(const Json::Value &value);
    virtual void to_impl(Json::Value &value) const;

protected:
    virtual Changed changed_impl(const DefinitionBase &other) const;
    virtual bool needsRanges_impl() const { return false; }
};

/** Semantic world description to tiled geodata generator.
 */
struct GeodataSemanticTiled : public GeodataSemanticBase {

    static constexpr Resource::Generator::Type type
        = Resource::Generator::Type::geodata;
    static constexpr char driverName[] = "geodata-semantic-tiled";

    GeodataSemanticTiled() = default;
};

} // namespace resource

#endif // mapproxy_definition_geodata_semantic_hpp_included_
