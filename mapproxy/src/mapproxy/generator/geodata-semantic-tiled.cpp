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

#include "jsoncpp/as.hpp"
#include "jsoncpp/io.hpp"

#include "semantic/io.hpp"
#include "semantic/gpkg.hpp"
#include "semantic/featurelayers.hpp"

#include "vts-libs/storage/fstreams.hpp"
#include "vts-libs/registry/json.hpp"

#include "../support/tileindex.hpp"
#include "../support/revision.hpp"
#include "../support/geo.hpp"
#include "../support/position.hpp"

#include "geodata-semantic-tiled.hpp"
#include "factory.hpp"
#include "metatile.hpp"
#include "files.hpp"
#include "../gdalsupport/workrequest.hpp"

namespace ba = boost::algorithm;
namespace fs = boost::filesystem;
namespace vr = vtslibs::registry;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<GeodataSemanticTiled>(params);
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType<GeodataSemanticTiled>(std::make_shared<Factory>());
});

} // namespace

GeodataSemanticTiled::GeodataSemanticTiled(const Params &params)
    : Generator(params)
    , definition_(this->resource().definition<Definition>())
    , dem_(absoluteDataset(definition_.dem.dataset + "/dem")
           , definition_.dem.geoidGrid)
    , styleUrl_(definition_.styleUrl)
    , dataset_(absoluteDataset(definition_.dataset))
    , physicalSrs_
      (vr::system.srs(resource().referenceFrame->model.physicalSrs))
{
    if (definition_.format != geo::VectorFormat::geodataJson) {
        LOGTHROW(err1, std::runtime_error)
            << "Unsupported output vector format <"
            << definition_.format << ">.";
    }
    if (const auto *config = boost::get<geo::vectorformat::GeodataConfig>
        (&definition_.formatConfig))
    {
        geodataConfig_ = *config;
    } else {
        LOGTHROW(err1, std::runtime_error)
            << "Missing configuration for vector format <"
            << definition_.format << ">.";
    }
    geo::GeoDataset::open(dem_.dataset);

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
        // map delivery index
        auto deliveryIndexPath(root() / "delivery.index");
        index_ = boost::in_place(referenceFrame().metaBinaryOrder
                                 , deliveryIndexPath);
        return;
    } catch (const std::exception &e) {
        // not ready
    }

    LOG(info1) << "Generator for <" << id() << "> not ready.";
}

void GeodataSemanticTiled::prepare_impl(Arsenal&)
{
    LOG(info2) << "Preparing <" << id() << ">.";

    const auto &r(resource());

    {
        semantic::GeoPackage gpkg(dataset_);
        const auto extents(gpkg.extents());

        auto position
            (positionFromPoints
             (referenceFrame(), gpkg.srs(), math::center(extents)
              , [&](const auto &callback) {
                 for (const auto &p : math::vertices(extents)) {
                     callback(p);
                 }
             }));
    }

    // try to open datasets
    geo::GeoDataset::open(dem_.dataset);
    geo::GeoDataset::open(dem_.dataset + ".min");
    geo::GeoDataset::open(dem_.dataset + ".max");

    // prepare tile index
    {
        vts::tileset::Index index(referenceFrame().metaBinaryOrder);
        prepareTileIndex(index
                         , (absoluteDataset(definition_.dem.dataset)
                            + "/tiling." + r.id.referenceFrame)
                         , r);

        // save it all
        vts::tileset::saveTileSetIndex(index, root() / "tileset.index");

        const auto deliveryIndexPath(root() / "delivery.index");
        // convert it to delivery index (using a temporary file)
        const auto tmpPath(utility::addExtension
                           (deliveryIndexPath, ".tmp"));
        mmapped::TileIndex::write(tmpPath, index.tileIndex);
        fs::rename(tmpPath, deliveryIndexPath);

        index_ = boost::in_place(referenceFrame().metaBinaryOrder
                                 , deliveryIndexPath);
    }
}

vr::FreeLayer GeodataSemanticTiled::freeLayer(ResourceRoot root) const
{
    const auto &res(resource());

    vr::FreeLayer fl;
    fl.id = res.id.fullId();
    fl.type = vr::FreeLayer::Type::geodataTiles;

    auto &def(fl.createDefinition<vr::FreeLayer::GeodataTiles>());
    def.metaUrl = prependRoot
        (utility::format("{lod}-{x}-{y}.meta?gr=%d%s"
                         , vts::MetaTile::currentVersion()
                         , RevisionWrapper(res.revision, "&"))
         , resource(), root);
    def.geodataUrl = prependRoot
        (utility::format("{lod}-{x}-{y}.geo%s"
                         , RevisionWrapper(res.revision, "?"))
         , resource(), root);
    def.style = styleUrl_;

    def.displaySize = definition_.displaySize;
    def.lodRange = res.lodRange;
    def.tileRange = res.tileRange;
    fl.credits = asInlineCredits(res);
    def.options = definition_.options;

    // done
    return fl;
}

vts::MapConfig GeodataSemanticTiled::mapConfig_impl(ResourceRoot root) const
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
        // TODO: store metadata
        // mapConfig.position = metadata_.position;
    }

    // browser options (must be Json::Value!); overrides browser options from
    // surface's introspection
    if (!definition_.introspection.browserOptions.empty()) {
        mapConfig.browserOptions = definition_.introspection.browserOptions;
    }

    // done
    return mapConfig;
}

Generator::Task
GeodataSemanticTiled::generateFile_impl(const FileInfo &fileInfo
                                        , Sink &sink) const
{
    GeodataFileInfo fi(fileInfo, true, definition_.format);

    // check for valid tileId
    switch (fi.type) {
    case GeodataFileInfo::Type::geo:
        return[=](Sink &sink, Arsenal &arsenal) {
            generateGeodata(sink, fi, arsenal);
        };
        break;

    case GeodataFileInfo::Type::metatile:
        {
        return[=](Sink &sink, Arsenal &arsenal) {
            generateMetatile(sink, fi, arsenal);
        };
        }

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

void GeodataSemanticTiled::generateMetatile(Sink &sink
                                            , const GeodataFileInfo &fi
                                            , Arsenal &arsenal) const
{
    // TODO: generate metadata from geodata
    sink.checkAborted();

    if (!index_->meta(fi.tileId)) {
        sink.error(utility::makeError<NotFound>("Metatile not found."));
        return;
    }

    auto metatile(metatileFromDem
                  (fi.tileId, sink, arsenal, resource()
                   , index_->tileIndex, dem_.dataset
                   , dem_.geoidGrid
                   , MaskTree(), definition_.displaySize));

    // write metatile to stream
    std::ostringstream os;
    metatile.save(os);
    sink.content(os.str(), fi.sinkFileInfo());
}

struct MemoryBlock {
    const char *data;
    std::size_t size;

    typedef std::shared_ptr<MemoryBlock> pointer;

    MemoryBlock(const char *data = nullptr, std::size_t size = 0)
        : data(data), size(size)
    {}

    static MemoryBlock* allocate(ManagedBuffer &mb
                                 , const char *data, std::size_t size);
};

MemoryBlock* MemoryBlock::allocate(ManagedBuffer &mb
                                   , const char *data, std::size_t size)
{
    auto *raw(static_cast<char*>
              (mb.allocate_aligned
               (sizeof(MemoryBlock) + size, alignof(MemoryBlock))));

    // poiter to output data
    auto *dataPtr(raw + sizeof(MemoryBlock));

    // copy data into block
    std::copy(data, data + size, dataPtr);

    return new (raw) MemoryBlock(dataPtr, size);
}

class SemanticJob : public WorkRequest {
public:
    SemanticJob(const WorkRequestParams &p
                , const std::string &dataset
                , const geo::SrsDefinition &outputSrs
                , bool outputAdjustVertical
                , const geo::SrsDefinition &srs
                , const math::Extents2 &extents
                , int lod
                , const geo::vectorformat::GeodataConfig &geodataConfig)
        : WorkRequest(p.sm)
        , rawTile_()
        , dataset_(dataset.data(), dataset.size(), p.sm.get_allocator<char>())
        , outputSrs_(outputSrs, p.sm)
        , outputAdjustVertical_(outputAdjustVertical)
        , srs_(srs, p.sm)
        , extents_(extents)
        , lod_(lod)
        , geodataConfig_(geodataConfig)
    {}

    ~SemanticJob() {
        if (rawTile_) { sm().deallocate(rawTile_); }
    }

    virtual void process(Mutex&, DatasetCache&) {
        const std::string dataset(dataset_.data(), dataset_.size());
        auto &ds(openDataset(dataset));

        semantic::GeoPackage::Query query;
        query.extents = extents_;
        query.srs = srs_;

        auto world(ds.world(query));

        // TODO: add meshconfig
        auto fl(semantic::featureLayers(world, {}, lod_));
        fl.transform(outputSrs_, outputAdjustVertical_);

        {
            std::ostringstream os;
            os.precision(15);
            fl.dumpVTSGeodata(os, geodataConfig_.resolution);

            const auto str(os.str());
            rawTile_ = MemoryBlock::allocate(sm(), str.data(), str.size());
        }
    }

    virtual Response response(Lock&) {
        if (!rawTile_) {
            LOGTHROW(err2, std::runtime_error)
                << "No tile generated";
        }

        MemoryBlock::pointer tile(rawTile_, [&sm=sm()](MemoryBlock *tile)
        {
            // deallocate data
            sm.deallocate(tile);
        });
        rawTile_ = nullptr;

        return tile;
    }

    /** Destroys this object. Only to be called from warper machinery.
     */
    virtual void destroy() { sm().destroy_ptr(this); }

private:
    using GeoPackageCache = std::map<std::string, semantic::GeoPackage>;
    static GeoPackageCache cache_;

    static const semantic::GeoPackage& openDataset(const std::string &path) {
        auto fcache(cache_.find(path));
        if (fcache != cache_.end()) { return fcache->second; }
        return cache_.emplace(path, path).first->second;
    }

    MemoryBlock *rawTile_;

    String dataset_;
    ShSrsDefinition outputSrs_;
    bool outputAdjustVertical_;
    ShSrsDefinition srs_;
    math::Extents2 extents_;
    int lod_;
    geo::vectorformat::GeodataConfig geodataConfig_;
};

SemanticJob::GeoPackageCache SemanticJob::cache_;

MemoryBlock::pointer
semantic2GeodataTile(Arsenal &arsenal, Aborter &aborter
                     , const std::string &dataset
                     , const geo::SrsDefinition &outputSrs
                     , bool outputAdjustVertical
                     , const geo::SrsDefinition &srs
                     , const math::Extents2 &extents
                     , int lod
                     , const geo::vectorformat::GeodataConfig &geodataConfig)
{
    auto response
        (arsenal.warper.job([&](const WorkRequestParams &params) {
                return params.sm.construct<SemanticJob>
                    (bi::anonymous_instance)
                    (params, dataset, outputSrs, outputAdjustVertical
                     , srs, extents, lod, geodataConfig);
            }, aborter));

    return std::static_pointer_cast<MemoryBlock>(response);
}

void GeodataSemanticTiled::generateGeodata(Sink &sink
                                           , const GeodataFileInfo &fi
                                           , Arsenal &arsenal) const
{
    const auto &tileId(fi.tileId);
    // TODO: get availability from geodata
    auto flags(index_->tileIndex.get(tileId));
    if (!vts::TileIndex::Flag::isReal(flags)) {
        sink.error(utility::makeError<NotFound>("No geodata for this tile."));
        return;
    }

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.productive()) {
        sink.error(utility::makeError<NotFound>
                   ("TileId outside of valid reference frame tree."));
        return;
    }

    auto tile(semantic2GeodataTile
              (arsenal, sink
               , dataset_.string()
               , physicalSrs_.srsDef, physicalSrs_.adjustVertical()
               , nodeInfo.srsDef(), nodeInfo.extents()
               , definition_.lod, geodataConfig_));

    sink.content(tile->data, tile->size, fi.sinkFileInfo(), true);
}

} // namespace generator
