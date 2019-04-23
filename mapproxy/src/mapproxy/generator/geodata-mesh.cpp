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

#include <algorithm>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"
#include "utility/format.hpp"
#include "utility/path.hpp"

#include "geo/featurelayers.hpp"
#include "geometry/meshop.hpp"

#include "vts-libs/storage/fstreams.hpp"

#include "../support/revision.hpp"

#include "geodata-mesh.hpp"
#include "factory.hpp"
#include "files.hpp"

namespace ba = boost::algorithm;
namespace fs = boost::filesystem;
namespace vr = vtslibs::registry;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<GeodataMesh>(params);
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType<GeodataMesh>(std::make_shared<Factory>());
});

} // namespace

GeodataMesh::GeodataMesh(const Params &params)
    : Generator(params)
    , definition_(this->resource().definition<Definition>())
    , styleUrl_(definition_.styleUrl)
    , dataPath_(root() / "geodata")
{
    if (styleUrl_.empty()) {
        styleUrl_ = "style.json";
    } else if (ba::istarts_with(styleUrl_, "file:")) {
        // pseudo file URL
        stylePath_ = absoluteDataset(styleUrl_.substr(5));
        styleUrl_ = "style.json";
    }

    // load geodata only if there is no enforced change
    if (changeEnforced()) {
        LOG(info1) << "Generator for <" << id() << "> not ready.";
        return;
    }

    try {
        metadata_ = geo::heightcoding::loadMetadata
            (root() / "metadata.json");
        if (fs::file_size(dataPath_) == metadata_.fileSize) {
            // valid file
            makeReady();
            return;
        }
        LOG(info1) << "Sizes differ, regenerate.";
    } catch (const std::exception &e) { /* not ready */ }

    LOG(info1) << "Generator for <" << id() << "> not ready.";
}

namespace {

geo::FeatureLayers mesh2fl(const geometry::Mesh &mesh
                           , const geo::SrsDefinition &srs
                           , bool adjustVertical
                           , const math::Point3 &center)
{
    geo::FeatureLayers fl;

    fl.layers.emplace_back("mesh", srs);
    auto &l(fl.layers.back());
    l.adjustVertical = adjustVertical;

    geo::FeatureLayers::Features::Surface s(1, "mesh", {});

    s.vertices.resize(mesh.vertices.size());
    std::transform(mesh.vertices.begin(), mesh.vertices.end()
                   , s.vertices.begin()
                   , [&center](const math::Point3 &p) -> math::Point3 {
                       return p + center;
                   });

    for (const auto &face : mesh.faces) {
        s.surface.emplace_back();
        auto &p(s.surface.back());
        p(0) = face.a;
        p(1) = face.b;
        p(2) = face.c;
    }

    l.features.addSurface(s);

    return fl;
}

} // namespace

void GeodataMesh::prepare_impl(Arsenal&)
{
    const auto dataset(absoluteDataset(definition_.dataset));

    // TODO: allow ply as well
    auto fl(mesh2fl(geometry::loadObj(dataset)
                    , definition_.srs, definition_.adjustVertical
                    , definition_.center));

    // get physical srs
    const auto srs(vr::system.srs
                   (resource().referenceFrame->model.physicalSrs));

    fl.transform(srs.srsDef, srs.adjustVertical());

    if (const auto extents = fl.boundingBox()) {
        metadata_.extents = *extents;
    }

    {
        utility::ofstreambuf f(dataPath_.string());
        f.precision(15);

        switch (definition_.format) {
        case geo::VectorFormat::geodataJson:
            if (const auto *c = boost::get<geo::vectorformat::GeodataConfig>
                (&definition_.formatConfig))
            {
                fl.dumpVTSGeodata(f, c->resolution);
            } else {
                LOGTHROW(err1, std::runtime_error)
                    << "Missing configuration for vector format <"
                    << definition_.format << ">.";
            }
            break;

        default:
            // unsupported
            LOGTHROW(err1, std::runtime_error)
                << "Unsupported output vector format <"
                << definition_.format << ">.";
        }

        f.close();
    }

    metadata_.fileSize = fs::file_size(dataPath_);
    geo::heightcoding::saveMetadata(root() / "metadata.json", metadata_);
}

vr::FreeLayer GeodataMesh::freeLayer(ResourceRoot root) const
{
    const auto &res(resource());

    vr::FreeLayer fl;
    fl.id = res.id.fullId();
    fl.type = vr::FreeLayer::Type::geodata;

    auto &def(fl.createDefinition<vr::FreeLayer::Geodata>());
    def.extents = metadata_.extents;
    def.displaySize = definition_.displaySize;
    def.label = res.comment;
    def.geodata = prependRoot
        (utility::format("geo%s", RevisionWrapper(res.revision, "?"))
         , res, root);
    def.style = styleUrl_;
    fl.credits = asInlineCredits(res);
    def.options = definition_.options;

    // done
    return fl;
}

vts::MapConfig GeodataMesh::mapConfig_impl(ResourceRoot root) const
{
    const auto &res(resource());

    vts::MapConfig mapConfig;
    mapConfig.referenceFrame = *res.referenceFrame;
    mapConfig.srs = vr::listSrs(*res.referenceFrame);

    // add free layer into list of free layers
    mapConfig.freeLayers.add
        (vr::FreeLayer
         (res.id.fullId()
          , prependRoot(std::string("freelayer.json"), resource(), root)));

    // add free layer into view
    mapConfig.view.freeLayers[res.id.fullId()];

    if (definition_.introspection.surface) {
        if (auto other = otherGenerator
            (Resource::Generator::Type::surface
             , addReferenceFrame(*definition_.introspection.surface
                                 , referenceFrameId())))
        {
            mapConfig.merge(other->mapConfig
                            (resolveRoot(resource(), other->resource())));
        }
    }

    // browser options (must be Json::Value!); overrides browser options from
    // surface's introspection
    if (!definition_.introspection.browserOptions.empty()) {
        mapConfig.browserOptions = definition_.introspection.browserOptions;
    }

    // done
    return mapConfig;
}

Generator::Task GeodataMesh::generateFile_impl(const FileInfo &fileInfo
                                               , Sink &sink) const
{
    GeodataFileInfo fi(fileInfo, false, definition_.format);

    // check for valid tileId
    switch (fi.type) {
    case GeodataFileInfo::Type::geo:
        return[=](Sink &sink, Arsenal &arsenal) {
            generateGeodata(sink, fi, arsenal);
        };
        break;

    case GeodataFileInfo::Type::config: {
        std::ostringstream os;
        mapConfig(os, ResourceRoot::none);
        sink.content(os.str(), fi.sinkFileInfo());
        break;
    }

    case GeodataFileInfo::Type::definition: {
        std::ostringstream os;
        vr::saveFreeLayer(os, freeLayer(ResourceRoot::none));
        sink.content(os.str(), fi.sinkFileInfo());
        break;
    }

    case GeodataFileInfo::Type::support:
        supportFile(*fi.support, sink, fi.sinkFileInfo());
        break;

    case GeodataFileInfo::Type::registry:
        sink.content(vs::fileIStream
                      (fi.registry->contentType, fi.registry->path)
                     , FileClass::registry);
        break;

    case GeodataFileInfo::Type::style:
        if (stylePath_.empty()) {
            // return internal file
            supportFile(files::defaultMeshStyle, sink, fi.sinkFileInfo());
        } else {
            // return external file
            sink.content(vs::fileIStream
                         (files::defaultMeshStyle.contentType, stylePath_)
                         , FileClass::config);
        }
        break;

    default:
        sink.error(utility::makeError<NotFound>("Not Found."));
        break;
    }

    return {};
}

void GeodataMesh::generateGeodata(Sink &sink, const GeodataFileInfo &fi
                                  , Arsenal &) const
{
    sink.content(vs::fileIStream(fi.sinkFileInfo().contentType.c_str()
                                 , dataPath_), FileClass::data);
}

} // namespace generator
