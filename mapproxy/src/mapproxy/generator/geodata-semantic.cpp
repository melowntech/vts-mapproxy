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

#include "math/transform.hpp"

#include "geo/featurelayers.hpp"
#include "geometry/meshop.hpp"

#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "semantic/io.hpp"
#include "semantic/mesh.hpp"

#include "vts-libs/storage/fstreams.hpp"
#include "vts-libs/registry/json.hpp"

#include "../support/revision.hpp"
#include "../support/geo.hpp"

#include "geodata-semantic.hpp"
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
        return std::make_shared<GeodataSemantic>(params);
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType<GeodataSemantic>(std::make_shared<Factory>());
});

void build(Json::Value &value, const GeodataSemantic::Metadata &metadata)
{
    auto &extents(value["extents"] = Json::arrayValue);
    extents.append(metadata.extents.ll(0));
    extents.append(metadata.extents.ll(1));
    extents.append(metadata.extents.ll(2));
    extents.append(metadata.extents.ur(0));
    extents.append(metadata.extents.ur(1));
    extents.append(metadata.extents.ur(2));

    value["fileSize"] = int(metadata.fileSize);
    value["position"] = vr::asJson(metadata.position);
}

void parse(GeodataSemantic::Metadata &metadata, const Json::Value &value)
{
    const auto &extents(Json::check(value["extents"], Json::arrayValue));
    metadata.extents.ll(0) = extents[0].asDouble();
    metadata.extents.ll(1) = extents[1].asDouble();
    metadata.extents.ll(2) = extents[2].asDouble();
    metadata.extents.ur(0) = extents[3].asDouble();
    metadata.extents.ur(1) = extents[4].asDouble();
    metadata.extents.ur(2) = extents[5].asDouble();

    Json::get(metadata.fileSize, value, "fileSize");

    if (value.isMember("position")) {
        metadata.position = vr::positionFromJson(value["position"]);
    }
}

GeodataSemantic::Metadata loadMetadata(const boost::filesystem::path &path)
{
    LOG(info1) << "Loading heightcoding::Metadata from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
    } catch (const std::exception &e) {
        LOGTHROW(err1, std::runtime_error)
            << "Unable to load heightcoding::Metadata from " << path << ".";
    }
    // load json
    auto content(Json::read<std::runtime_error>
                 (f, path, "heightcoding::Metadata"));

    GeodataSemantic::Metadata metadata;
    parse(metadata, content);
    f.close();
    return metadata;
}

void saveMetadata(const boost::filesystem::path &path
                  , const GeodataSemantic::Metadata &metadata)
{
    LOG(info1) << "Saving heightcoding::Metadata into " << path  << ".";
    std::ofstream f;
    try {
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path.string(), std::ios_base::out);
    } catch (const std::exception &e) {
        LOGTHROW(err1, std::runtime_error)
            << "Unable to save heightcoding::Metadata into "
            << path << ".";
    }

    Json::Value content;
    build(content, metadata);
    f.precision(15);
    Json::write(f, content);

    f.close();
}

} // namespace

GeodataSemantic::GeodataSemantic(const Params &params)
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
        metadata_ = loadMetadata(root() / "metadata.json");
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

geo::FeatureLayers mesh2fl(const geometry::Mesh::list meshes
                           , const std::vector<std::string> &materials
                           , const geo::SrsDefinition &srs
                           , bool adjustVertical)
{
    geo::FeatureLayers fl;

    fl.layers.emplace_back("mesh", srs);
    auto &l(fl.layers.back());
    l.adjustVertical = adjustVertical;

    int fid(0);
    for (const auto &mesh : meshes) {
        geo::FeatureLayers::Features::Properties props;
        const auto &meshName(materials[mesh.faces.front().imageId]);
        props["name"] = meshName;
        geo::FeatureLayers::Features::Surface s(++fid, meshName, props);

        s.vertices = mesh.vertices;

        for (const auto &face : mesh.faces) {
            s.surface.emplace_back();
            auto &p(s.surface.back());
            p(0) = face.a;
            p(1) = face.b;
            p(2) = face.c;
        }

        l.features.addSurface(s);
    }

    return fl;
}

class LayerBuilder {
public:
    LayerBuilder(const semantic::World &world)
        : world_(world), fid_()
        , materials_(semantic::materials())
    {
        semantic::mesh(world, semantic::MeshConfig()
                       , [this](auto&&... args) {
                           this->mesh(std::forward<decltype(args)>(args)...);
                       }
                       , 2);
    }

    geo::FeatureLayers featureLayers() {
        geo::FeatureLayers fl;
        for (auto &item : layers_) {
            fl.layers.emplace_back(std::move(item.second));
        }
        return fl;
    }

private:
    using Layer = geo::FeatureLayers::Layer;
    using LayerMap = std::map<semantic::Class, Layer>;
    using Features = geo::FeatureLayers::Features;

    template <typename Entity>
    void mesh(const Entity &entity, const geometry::Mesh &mesh)
    {
        auto &l(layer(entity.cls));

        for (const auto &sm : geometry::splitById(mesh)) {
            // TODO: get more properties from the source
            Features::Properties props;
            props["material"] = materials_[sm.faces.front().imageId];

            // add surface
            auto &s(l.features.addSurface(++fid_, entity.id, props));
            s.vertices = sm.vertices;
            for (const auto &face : sm.faces) {
                s.surface.emplace_back(face.a, face.b, face.c);
            }
        }
    }

    Layer& layer(semantic::Class cls) {
        auto flayers(layers_.find(cls));
        if (flayers != layers_.end()) { return flayers->second; }

        const auto name(boost::lexical_cast<std::string>(cls));
        return layers_.emplace
            (std::piecewise_construct
             , std::forward_as_tuple(cls)
             , std::forward_as_tuple(name, world_.srs, true))
            .first->second;
    }

    const semantic::World &world_;
    LayerMap layers_;
    Features::Fid fid_;
    std::vector<std::string> materials_;
};

geo::FeatureLayers generateLayer(const semantic::World &world, bool simplified)
{
    if (simplified) {
        /** Generate mesh and split it by material (i.e. imageId); we expect the
         *  mesh to be optimized, i.e. no duplicate vertex exist.
         */
        return mesh2fl(geometry::splitById(semantic::mesh(world, {}, 2))
                       , semantic::materials(), world.srs, true);
    }

    return LayerBuilder(world).featureLayers();
}

} // namespace

void GeodataSemantic::prepare_impl(Arsenal&)
{
    const auto dataset(absoluteDataset(definition_.dataset));

    const auto world(semantic::load(dataset));

    auto fl(generateLayer(world, definition_.simplified));

    if (const auto extents = fl.boundingBox()) {
        // mesh center in navigation SRS
        const auto c(vts::CsConvertor
                     (world.srs
                      , resource().referenceFrame->model.navigationSrs)
                     (math::center(*extents)));

        auto &pos(metadata_.position);
        pos.type = vr::Position::Type::objective;
        pos.heightMode = vr::Position::HeightMode::floating;
        pos.position = c;
        pos.position[2] = 0.0; // floating -> zero
        pos.lookDown();
        pos.verticalFov = vr::Position::naturalFov();

        // compute vertical extent by taking a "photo" of physical data from
        // view's "camera"
        const auto trafo(makePlaneTrafo(referenceFrame(), pos.position));
        math::Extents2 cameraExtents(math::InvalidExtents{});
        fl.for_each_vertex([&](const math::Point3d &p)
        {
            math::update(cameraExtents, math::transform(trafo, p));
        });

        const auto cameraExtentsSize(math::size(cameraExtents));
        pos.verticalExtent = std::max(cameraExtentsSize.width
                                      , cameraExtentsSize.height);
    }

    // get physical srs
    const auto srs(vr::system.srs
                   (resource().referenceFrame->model.physicalSrs));
    fl.transform(srs.srsDef, srs.adjustVertical());

    // measure extents in physical SRS
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
    saveMetadata(root() / "metadata.json", metadata_);
}

vr::FreeLayer GeodataSemantic::freeLayer(ResourceRoot root) const
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

vts::MapConfig GeodataSemantic::mapConfig_impl(ResourceRoot root) const
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

    // override position
    if (definition_.introspection.position) {
        // user supplied
        mapConfig.position = *definition_.introspection.position;
    } else {
        // calculated
        mapConfig.position = metadata_.position;
    }

    // browser options (must be Json::Value!); overrides browser options from
    // surface's introspection
    if (!definition_.introspection.browserOptions.empty()) {
        mapConfig.browserOptions = definition_.introspection.browserOptions;
    }

    // done
    return mapConfig;
}

Generator::Task GeodataSemantic::generateFile_impl(const FileInfo &fileInfo
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

void GeodataSemantic::generateGeodata(Sink &sink, const GeodataFileInfo &fi
                                      , Arsenal &) const
{
    sink.content(vs::fileIStream(fi.sinkFileInfo().contentType.c_str()
                                 , dataPath_), FileClass::data);
}

} // namespace generator
