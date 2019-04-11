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

#ifndef mapproxy_definition_surface_hpp_included_
#define mapproxy_definition_surface_hpp_included_

#include <boost/optional.hpp>
#include <boost/filesystem.hpp>

#include "geo/geodataset.hpp"

#include "../support/geo.hpp"
#include "../resource.hpp"

#include "../heightfunction.hpp"

#include "../support/introspection.hpp"

// fwd
namespace Json { class Value; }
namespace boost { namespace python { class dict; } }

namespace resource {

// surface formats

class Surface : public DefinitionBase {
public:
    struct Introspection {
        introspection::Layers tms;
        introspection::Layers geodata;
        boost::optional<vr::Position> position;
        boost::any browserOptions;

        bool empty() const;
        bool operator!=(const Introspection &other) const;
    };

    boost::optional<double> nominalTexelSize;
    boost::optional<vts::Lod> mergeBottomLod;
    HeightFunction::pointer heightFunction;
    Introspection introspection;

    void parse(const Json::Value &value);
    void build(Json::Value &value) const;
    void parse(const boost::python::dict &value);

    virtual boost::optional<std::string> getGeoidGrid() const = 0;

    static constexpr Resource::Generator::Type type
        = Resource::Generator::Type::surface;

protected:
    virtual Changed changed_impl(const DefinitionBase &other) const;
};

struct SurfaceSpheroid : public Surface {
    unsigned int textureLayerId;
    boost::optional<std::string> geoidGrid;

    SurfaceSpheroid() : textureLayerId() {}

    static constexpr char driverName[] = "surface-spheroid";

private:
    virtual void from_impl(const boost::any &value);
    virtual void to_impl(boost::any &value) const;
    virtual Changed changed_impl(const DefinitionBase &other) const;
    virtual boost::optional<std::string> getGeoidGrid() const {
        return geoidGrid;
    }
};

struct SurfaceDem : public Surface {
    DemDataset dem;
    boost::optional<boost::filesystem::path> mask;
    unsigned int textureLayerId;
    boost::optional<std::string> heightcodingAlias;

    SurfaceDem() : textureLayerId() {}

    static constexpr char driverName[] = "surface-dem";

private:
    virtual void from_impl(const boost::any &value);
    virtual void to_impl(boost::any &value) const;
    virtual Changed changed_impl(const DefinitionBase &other) const;
    virtual boost::optional<std::string> getGeoidGrid() const {
        return dem.geoidGrid;
    }
};

} // namespace resource

#endif // mapproxy_definition_surface_hpp_included_

