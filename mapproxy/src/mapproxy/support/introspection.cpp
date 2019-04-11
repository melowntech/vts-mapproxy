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

#include "dbglog/dbglog.hpp"

#include "vts-libs/vts/tileset/properties.hpp"
#include "vts-libs/vts/urltemplate.hpp"

#include "../error.hpp"
#include "introspection.hpp"

namespace fs = boost::filesystem;
namespace vr = vtslibs::registry;

namespace introspection {

boost::optional<RemoteLayer>
remote(Resource::Generator::Type type, const Layer &layer
       , const Resource &resource, const FindResource &findResource)
{
    class Visitor
        : public boost::static_visitor<boost::optional<RemoteLayer>>
    {
    public:
        Visitor(Resource::Generator::Type type, const Resource &resource
                , const FindResource &findResource)
            : type_(type), resource_(resource), findResource_(findResource)
        {}

        boost::optional<RemoteLayer> operator()(const LocalLayer &l) const {
            const auto *other(findResource_(type_, l));
            if (!other) { return boost::none; }

            const auto otherId(other->id.fullId());
            const auto resdiff(resolveRoot(resource_, *other));

            const fs::path path
                (prependRoot(fs::path(), *other, resdiff)
                 / ((type_ == Resource::Generator::Type::geodata)
                    ? "freelayer.json" : "boundlayer.json"));

            return RemoteLayer(resource_.id.fullId(), path.string());
        }

        boost::optional<RemoteLayer> operator()(RemoteLayer l) const {
            // expand reference frame in URL
            vts::UrlTemplate::Vars vars;
            vars.rf = resource_.id.referenceFrame;
            l.url = vts::UrlTemplate(l.url)(vars);
            return l;
        }

    private:
        Resource::Generator::Type type_;
        const Resource &resource_;
        const FindResource &findResource_;
    } visitor(type, resource, findResource);

    return boost::apply_visitor(visitor, layer);
}

void add(vts::ExtraTileSetProperties &extra, Resource::Generator::Type type
         , const Layer &layer, const Resource &resource
         , const FindResource &findResource)
{
    if (const auto &r = remote(type, layer, resource, findResource)) {
        switch (type) {
        case Resource::Generator::Type::tms: {
            vr::BoundLayer bl(r->id, r->url);
            extra.boundLayers.add(bl);
            extra.view.surfaces[resource.id.fullId()].emplace_back(bl.id);
        } break;

        case Resource::Generator::Type::geodata: {
            vr::FreeLayer fl(r->id, r->url);
            extra.freeLayers.add(fl);
            extra.view.freeLayers[fl.id];
        } break;

        default: break; // ignore
        }
    }
}

} // namespace introspection
