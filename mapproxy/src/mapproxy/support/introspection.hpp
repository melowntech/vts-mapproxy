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

#ifndef mapproxy_support_introspection_hpp_included_
#define mapproxy_support_introspection_hpp_included_

#include <vector>
#include <functional>

#include <boost/variant.hpp>

#include "../resource.hpp"

// fwd
namespace Json { class Value; }
namespace vtslibs { namespace vts { struct ExtraTileSetProperties; } }

namespace introspection {

typedef Resource::Id LocalLayer;

struct RemoteLayer {
    std::string id;
    std::string url;

    RemoteLayer(const std::string &id = std::string()
                , const std::string &url = std::string())
        : id(id), url(url) {}

    bool operator<(const RemoteLayer &o) const;
    bool operator==(const RemoteLayer &o) const;
};

typedef boost::variant<LocalLayer, RemoteLayer> Layer;
typedef std::vector<Layer> Layers;

typedef std::function<const Resource*(Resource::Generator::Type
                                      , const Resource::Id&)> FindResource;

void add(vts::ExtraTileSetProperties &extra, Resource::Generator::Type type
         , const Layer &layer, const Resource &resource
         , const FindResource &findResource);

void add(vts::ExtraTileSetProperties &extra, Resource::Generator::Type type
         , const Layers &layers, const Resource &resource
         , const FindResource &findResource);

boost::optional<RemoteLayer>
remote(Resource::Generator::Type type, const Layer &layer
       , const Resource &resource, const FindResource &findResource);

/** ------------------------------------------------------------------------
 *  Parsing
 */
Layers layersFrom(const Json::Value &introspection, const std::string &key);

void layersTo(Json::Value &introspection, const std::string &key
              , const Layers &layers);

Resource::OptId idFrom(const Json::Value &introspection
                       , const std::string &key);

void idTo(Json::Value &introspection, const std::string &key
          , const Resource::OptId &resource);

// inlines

inline bool RemoteLayer::operator<(const RemoteLayer &o) const {
    if (id < o.id) { return true; }
    else if (o.id < id) { return false; }

    return url < o.url;
}

inline bool RemoteLayer::operator==(const RemoteLayer &o) const {
    return ((id == o.id) && (url == o.url));
}

inline void add(vts::ExtraTileSetProperties &extra
                , Resource::Generator::Type type, const Layers &layers
                , const Resource &resource, const FindResource &findResource)
{
    for (const auto &layer : layers) {
        add(extra, type, layer, resource, findResource);
    }
}

} // namespace introspection

#endif // mapproxy_support_serialization_hpp_included_
