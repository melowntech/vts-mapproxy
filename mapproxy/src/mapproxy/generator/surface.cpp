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

#include <new>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/raise.hpp"
#include "utility/path.hpp"
#include "utility/gzipper.hpp"

#include "imgproc/rastermask/cvmat.hpp"
#include "imgproc/png.hpp"

#include "vts-libs/registry/io.hpp"
#include "vts-libs/storage/fstreams.hpp"
#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/tileset/config.hpp"
#include "vts-libs/vts/metatile.hpp"
#include "vts-libs/vts/csconvertor.hpp"
#include "vts-libs/vts/math.hpp"
#include "vts-libs/vts/mesh.hpp"
#include "vts-libs/vts/opencv/navtile.hpp"
#include "vts-libs/vts/types2d.hpp"
#include "vts-libs/vts/qtree-rasterize.hpp"
#include "vts-libs/vts/2d.hpp"
#include "vts-libs/vts/debug.hpp"
#include "vts-libs/vts/mapconfig.hpp"
#include "vts-libs/vts/service.hpp"

#include "../error.hpp"
#include "../support/metatile.hpp"
#include "../support/mesh.hpp"
#include "../support/srs.hpp"
#include "../support/grid.hpp"
#include "../support/python.hpp"
#include "../support/serialization.hpp"
#include "../support/mmapped/qtree-rasterize.hpp"
#include "../support/tilejson.hpp"
#include "../support/cesiumconf.hpp"
#include "../support/revision.hpp"
#include "../support/tms.hpp"

#include "surface.hpp"

namespace fs = boost::filesystem;
namespace bio = boost::iostreams;
namespace vr = vtslibs::registry;
namespace vs = vtslibs::storage;
namespace vts = vtslibs::vts;

namespace generator {

fs::path SurfaceBase::filePath(vts::File fileType) const
{
    switch (fileType) {
    case vts::File::config:
        return root() / "tileset.conf";
    case vts::File::tileIndex:
        return root() / "tileset.index";
    default: break;
    }

    throw utility::makeError<InternalError>("Unsupported file");
}

SurfaceBase::SurfaceBase(const Params &params)
    : Generator(params)
    , definition_(resource().definition<Definition>())
    , tms_(params.resource.referenceFrame->findExtension<vre::Tms>())
{}

bool SurfaceBase::loadFiles(const Definition &definition)
{
    if (changeEnforced()) {
        LOG(info1) << "Generator for <" << id() << "> not ready.";
        return false;
    }

    try {
        auto indexPath(filePath(vts::File::tileIndex));
        auto deliveryIndexPath(root() / "delivery.index");
        auto propertiesPath(filePath(vts::File::config));
        if (fs::exists(indexPath) && fs::exists(propertiesPath)) {
            // both paths exist -> ok
            properties_ = vts::tileset::loadConfig(propertiesPath);
            if (updateProperties(definition)) {
                // something changed in properties, update
                vts::tileset::saveConfig(filePath(vts::File::config)
                                         , properties_);
            }

            if (!fs::exists(deliveryIndexPath)) {
                // no delivery index -> create
                vts::tileset::Index index(referenceFrame().metaBinaryOrder);
                vts::tileset::loadTileSetIndex(index, indexPath);

                // convert it to delivery index (using a temporary file)
                const auto tmpPath(utility::addExtension
                                   (deliveryIndexPath, ".tmp"));
                mmapped::TileIndex::write(tmpPath, index.tileIndex);
                fs::rename(tmpPath, deliveryIndexPath);
            }

            // load delivery index
            index_ = boost::in_place(referenceFrame().metaBinaryOrder
                                     , deliveryIndexPath);
            makeReady();
            return true;
        }
    } catch (const std::exception &e) {
        // not ready
    }

    LOG(info1) << "Generator for <" << id() << "> not ready.";
    return false;
}

bool SurfaceBase::updateProperties(const Definition &def)
{
    bool changed(false);

    if (properties_.nominalTexelSize != def.nominalTexelSize) {
        properties_.nominalTexelSize = def.nominalTexelSize;
        changed = true;
    }

    if (def.mergeBottomLod) {
        if (properties_.mergeBottomLod != *def.mergeBottomLod) {
            properties_.mergeBottomLod = *def.mergeBottomLod;
            changed = true;
        }
    } else if (properties_.mergeBottomLod) {
        properties_.mergeBottomLod = 0;
        changed = true;
    }

    // update revision if changed
    if (resource().revision > properties_.revision) {
        properties_.revision = resource().revision;
        changed = true;
    }

    return changed;
}

Generator::Task SurfaceBase
::generateFile_impl(const FileInfo &fileInfo, Sink &sink) const
{
    // handle special case
    switch (fileInfo.interface.interface) {
    case GeneratorInterface::Interface::vts: break;
    case GeneratorInterface::Interface::terrain:
        return terrainInterface(fileInfo, sink);
    default:
        sink.error(utility::makeError<InternalError>
                   ("Surface resource has no <%s> interface."
                    , fileInfo.interface));
        return {};
    }

    SurfaceFileInfo fi(fileInfo);

    switch (fi.type) {
    case SurfaceFileInfo::Type::unknown:
        sink.error(utility::makeError<NotFound>("Unrecognized filename."));
        break;

    case SurfaceFileInfo::Type::definition: {
        auto fl(vts::freeLayer
                (vts::meshTilesConfig
                 (properties_, vts::ExtraTileSetProperties()
                  , prependRoot(fs::path(), resource(), ResourceRoot::none))));

        std::ostringstream os;
        vr::saveFreeLayer(os, fl);
        sink.content(os.str(), fi.sinkFileInfo());
        break;
    }

    case SurfaceFileInfo::Type::file: {
        switch (fi.fileType) {
        case vts::File::config: {
            switch (fi.flavor) {
            case vts::FileFlavor::regular: {
                std::ostringstream os;
                mapConfig(os, ResourceRoot::none);
                sink.content(os.str(), fi.sinkFileInfo());
                break;
            }

            case vts::FileFlavor::raw:
                sink.content(vs::fileIStream
                             (fi.fileType, filePath(vts::File::config))
                             , FileClass::unknown);
                break;

            case vts::FileFlavor::debug: {
                std::ostringstream os;
                const auto debug
                    (vts::debugConfig
                     (vts::meshTilesConfig
                      (properties_, vts::ExtraTileSetProperties()
                       , prependRoot(fs::path(), resource()
                                     , ResourceRoot::none))));
                vts::saveDebug(os, debug);
                sink.content(os.str(), fi.sinkFileInfo());
                break;
            }

            default:
                sink.error(utility::makeError<NotFound>
                           ("Unsupported file flavor %s.", fi.flavor));
                break;
            }
            break;
        }

        case vts::File::tileIndex:
            sink.content(vs::fileIStream
                          (fi.fileType, filePath(vts::File::tileIndex))
                         , FileClass::unknown);
            break;

        case vts::File::registry: {
            std::ostringstream os;
            save(os, resource().registry);
            sink.content(os.str(), fi.sinkFileInfo());
            break; }

        default:
            sink.error(utility::makeError<NotFound>("Not found"));
            break;
        }
        break;
    }

    case SurfaceFileInfo::Type::tile: {
        switch (fi.tileType) {
        case vts::TileFile::meta:
            return[=](Sink &sink, Arsenal &arsenal) {
                if (fi.flavor == vts::FileFlavor::debug) {
                    // debug metanode
                    generateDebugNode(fi.tileId, sink, fi, arsenal);
                } else {
                    // regular metatile
                    generateMetatile(fi.tileId, sink, fi, arsenal);
                }
            };

        case vts::TileFile::mesh:
            return[=](Sink &sink, Arsenal &arsenal) {
                generateMesh(fi.tileId, sink, fi, arsenal);
            };

        case vts::TileFile::atlas:
            sink.error(utility::makeError<NotFound>
                        ("No internal texture present."));
            break;

        case vts::TileFile::navtile:
            return[=](Sink &sink, Arsenal &arsenal) {
                generateNavtile(fi.tileId, sink, fi, arsenal);
            };
            break;

        case vts::TileFile::meta2d:
            return[=](Sink &sink, Arsenal &arsenal) {
                generate2dMetatile(fi.tileId, sink, fi, arsenal);
            };
            break;

        case vts::TileFile::mask:
            return[=](Sink &sink, Arsenal &arsenal) {
                generate2dMask(fi.tileId, sink, fi, arsenal);
            };
            break;

        case vts::TileFile::ortho:
            sink.error(utility::makeError<NotFound>
                        ("No orthophoto present."));
            break;

        case vts::TileFile::credits:
            return[=](Sink &sink, Arsenal &arsenal) {
                generateCredits(fi.tileId, sink, fi, arsenal);
            };
            break;
        }
        break;
    }

    case SurfaceFileInfo::Type::support:
        supportFile(*fi.support, sink, fi.sinkFileInfo());
        break;

    case SurfaceFileInfo::Type::registry:
        sink.content(vs::fileIStream
                      (fi.registry->contentType, fi.registry->path)
                     , FileClass::registry);
        break;

    case SurfaceFileInfo::Type::service:
        sink.content(vts::service::generate
                     (fi.serviceFile, fi.fileInfo.filename, fi.fileInfo.query)
                     , FileClass::data);
        break;

    default:
        sink.error(utility::makeError<InternalError>
                    ("Not implemented yet."));
    }

    return {};
}

void SurfaceBase::generateMesh(const vts::TileId &tileId
                               , Sink &sink
                               , const SurfaceFileInfo &fi
                               , Arsenal &arsenal) const
{
    auto flags(index_->tileIndex.get(tileId));
    if (!vts::TileIndex::Flag::isReal(flags)) {
        utility::raise<NotFound>("No mesh for this tile.");
    }

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.productive()) {
        utility::raise<NotFound>
            ("TileId outside of valid reference frame tree.");
    }

    // generate the actual mesh
    auto lm(generateMeshImpl(nodeInfo, sink, arsenal));

    // and add skirt
    addSkirt(lm.mesh, nodeInfo);

    const auto raw(fi.flavor == vts::FileFlavor::raw);

    // generate VTS mesh
    vts::Mesh mesh(false);
    if (!lm.mesh.vertices.empty()) {
        // local mesh is valid -> add as a submesh into output mesh
        auto &sm(addSubMesh(mesh, lm.mesh, nodeInfo, lm.geoidGrid));
        if (lm.textureLayerId) {
            sm.textureLayer = lm.textureLayerId;
        }

        if (raw) {
            // we are returning full mesh file -> generate coverage mask
            meshCoverageMask
                (mesh.coverageMask, lm.mesh, nodeInfo, lm.fullyCovered);
        }
    }

    // write mesh to stream
    std::stringstream os;
    auto sfi(fi.sinkFileInfo());
    if (raw) {
        vts::saveMesh(os, mesh);
    } else {
        vts::saveMeshProper(os, mesh);
        if (vs::gzipped(os)) {
            // gzip -> mesh
            sfi.addHeader("Content-Encoding", "gzip");
        }
    }

    sink.content(os.str(), sfi);
}

void SurfaceBase::generate2dMask(const vts::TileId &tileId
                                 , Sink &sink
                                 , const SurfaceFileInfo &fi
                                 , Arsenal &arsenal) const
{
    const auto debug(fi.flavor == vts::FileFlavor::debug);

    auto flags(index_->tileIndex.get(tileId));
    if (!vts::TileIndex::Flag::isReal(flags)) {
        if (debug) {
            return sink.error(utility::makeError<EmptyDebugMask>
                              ("No mesh for this tile."));
        }
        return sink.error(utility::makeError<NotFound>
                          ("No mesh for this tile."));
    }

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.productive()) {
        if (debug) {
            return sink.error(utility::makeError<EmptyDebugMask>
                              ("No mesh for this tile."));
        }
        return sink.error(utility::makeError<NotFound>
                          ("TileId outside of valid reference frame tree."));
    }

    // by default full watertight mesh
    vts::MeshMask mask;
    mask.createCoverage(true);

    if (!vts::TileIndex::Flag::isWatertight(flags)) {
        auto lm(generateMeshImpl(nodeInfo, sink, arsenal));
        meshCoverageMask
            (mask.coverageMask, lm.mesh, nodeInfo, lm.fullyCovered);
    }

    if (debug) {
        sink.content(imgproc::png::serialize
                     (vts::debugMask(mask.coverageMask, { 1 }), 9)
                     , fi.sinkFileInfo());
    } else {
        sink.content(imgproc::png::serialize
                     (vts::mask2d(mask.coverageMask, { 1 }), 9)
                     , fi.sinkFileInfo());
    }
}

void SurfaceBase::generate2dMetatile(const vts::TileId &tileId
                                     , Sink &sink
                                     , const SurfaceFileInfo &fi
                                     , Arsenal&) const

{
    sink.content(imgproc::png::serialize
                 (vts::meta2d(index_->tileIndex, tileId), 9)
                 , fi.sinkFileInfo());
}

void SurfaceBase::generateCredits(const vts::TileId&
                                  , Sink &sink
                                  , const SurfaceFileInfo &fi
                                  , Arsenal&) const
{
    vts::CreditTile creditTile;
    creditTile.credits = asInlineCredits(resource());

    std::ostringstream os;
    saveCreditTile(os, creditTile, true);
    sink.content(os.str(), fi.sinkFileInfo());
}

void SurfaceBase::generateDebugNode(const vts::TileId &tileId
                                    , Sink &sink
                                    , const SurfaceFileInfo &fi
                                    , Arsenal &) const
{
    // generate debug metanode
    const auto debugNode(vts::getNodeDebugInfo(index_->tileIndex, tileId));

    std::ostringstream os;
    vts::saveDebug(os, debugNode);
    sink.content(os.str(), fi.sinkFileInfo());
}

vts::ExtraTileSetProperties
SurfaceBase::extraProperties(const Definition &def) const
{
    vts::ExtraTileSetProperties extra;

    Resource::Id::list introspectionTmsList(def.introspection.tms);
    if (introspectionTmsList.empty()) {
        // defaults to patchwork
        introspectionTmsList.emplace_back(referenceFrameId()
                                          , systemGroup()
                                          , "tms-raster-patchwork");
    }

    for (const auto &tms : introspectionTmsList) {
        if (auto other = otherGenerator
            (Resource::Generator::Type::tms
             , addReferenceFrame(tms, referenceFrameId())))
        {
            // we have found tms resource, use it as a boundlayer
            const auto otherId(tms.fullId());
            const auto &otherResource(other->resource());
            const auto resdiff(resolveRoot(resource(), otherResource));

            const fs::path blPath
                (prependRoot(fs::path(), otherResource, resdiff)
                 / "boundlayer.json");

            extra.boundLayers.add(vr::BoundLayer(otherId, blPath.string()));

            extra.view.surfaces[id().fullId()]
                .push_back(vr::View::BoundLayerParams(otherId));
        };
    }

    for (const auto &geodata : def.introspection.geodata) {
        if (auto other = otherGenerator
            (Resource::Generator::Type::geodata
             , addReferenceFrame(geodata, referenceFrameId())))
        {
            // we have found geodata resource, use it as a boundlayer
            const auto otherId(geodata.fullId());
            const auto &otherResource(other->resource());
            const auto resdiff(resolveRoot(resource(), otherResource));

            const fs::path flPath
                (prependRoot(fs::path(), otherResource, resdiff)
                 / "freelayer.json");

            extra.freeLayers.add(vr::FreeLayer(otherId, flPath.string()));

            extra.view.freeLayers[otherId];
        };
    }

    if (def.introspection.position) {
        extra.position = *def.introspection.position;
    }

    // browser options (must be Json::Value!)
    extra.browserOptions = def.introspection.browserOptions;


    return extra;
}

const vre::Tms& SurfaceBase::getTms() const
{
    if (!tms_) {
        utility::raise<NotFound>
            ("Terrain provider interface disabled, no <tms> extension in "
             "reference frame <%s>.", referenceFrameId());
    }

    return *tms_;
}

Generator::Task SurfaceBase
::terrainInterface(const FileInfo &fileInfo, Sink &sink) const
{
    const auto &tms(getTms());

    TerrainFileInfo fi(fileInfo);

    switch (fi.type) {
    case TerrainFileInfo::Type::unknown:
        sink.error(utility::makeError<NotFound>("Unrecognized filename."));
        break;

    case TerrainFileInfo::Type::tile:
        return[=](Sink &sink, Arsenal &arsenal) {
            generateTerrain(fi.tileId, sink, fi, arsenal, tms);
        };

    case TerrainFileInfo::Type::definition:
        layerJson(sink, fi, tms);
        break;

    case TerrainFileInfo::Type::support:
        supportFile(*fi.support, sink, fi.sinkFileInfo());
        break;

    case TerrainFileInfo::Type::cesiumConf:
        cesiumConf(sink, fi, tms);
        break;

    default:
        sink.error(utility::makeError<InternalError>
                    ("Not implemented yet."));
    }

    return {};
}

void SurfaceBase::generateTerrain(const vts::TileId &tmsTileId
                                  , Sink &sink
                                  , const TerrainFileInfo &fi
                                  , Arsenal &arsenal
                                  , const vre::Tms &tms) const
{
    // remap id from TMS to VTS
    const auto tileId(tms2vts(tms.rootId, tms.flipY, tmsTileId));

    auto flags(index_->tileIndex.get(tileId));
    if (!vts::TileIndex::Flag::isReal(flags)) {
        utility::raise<NotFound>("No terrain for this tile.");
    }

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.productive()) {
        utility::raise<NotFound>
            ("TileId outside of valid reference frame tree.");
    }

    // generate the actual mesh
    auto lm(generateMeshImpl(nodeInfo, sink, arsenal));

    // write mesh to stream (gzipped)
    std::ostringstream os;
    qmf::save(qmfMesh(lm.mesh, nodeInfo
                      , (tms.physicalSrs ? *tms.physicalSrs
                         : referenceFrame().model.physicalSrs)
                      , lm.geoidGrid)
              , utility::Gzipper(os), fi.fileInfo.filename);

    auto sfi(fi.sinkFileInfo());
    sfi.addHeader("Content-Encoding", "gzip");
    sink.content(os.str(), sfi);
}

void SurfaceBase::layerJson(Sink &sink, const TerrainFileInfo &fi
                            , const vre::Tms &tms) const
{
    LayerJson layer;
    const auto &r(resource());

    layer.name = id().fullId();
    layer.description = r.comment;

    // use revision as major version (plus 1)
    layer.version.maj = r.revision + 1;
    layer.format = "quantized-mesh-1.0";
    layer.scheme = LayerJson::Scheme::tms;
    layer.tiles.push_back
        (utility::format("{z}-{x}-{y}.terrain%s"
                         , RevisionWrapper(r.revision, "?")));
    layer.projection = tms.projection;

    // fixed LOD range
    layer.zoom.min = r.lodRange.min - tms.rootId.lod;
    layer.zoom.max = r.lodRange.max - tms.rootId.lod;

    // invalidate extents, updated in this per-lod loop
    layer.bounds = math::Extents2(math::InvalidExtents{});

    // TODO: use tileindex instead of single tile range; should be OK for now
    for (const auto &range : vts::Ranges(r.lodRange, r.tileRange).ranges()) {
        layer.available.emplace_back();
        auto &current(layer.available.back());
        const auto tmsRange(vts2tms(tms.rootId, tms.flipY, range));
        current.push_back(tmsRange.range);

        const auto physicalSrs(tms.physicalSrs ? *tms.physicalSrs
                               : referenceFrame().model.physicalSrs);

        // treat current LOD as one gigantic metatile
        for (const auto &block : metatileBlocks(r, vts::TileId(range.lod, 0, 0)
                                                , range.lod, false))
        {
            const vts::CsConvertor conv(block.srs, physicalSrs);
            math::update(layer.bounds, conv(block.extents));
        }
    }

    if (!r.credits.empty()) {
        layer.attribution
            = boost::lexical_cast<std::string>
            (utility::join(html(asInlineCredits(r)), "<br/>"));
    }

    std::ostringstream os;
    save(layer, os);
    sink.content(os.str(), fi.sinkFileInfo());
}

void SurfaceBase::cesiumConf(Sink &sink, const TerrainFileInfo &fi
                             , const vre::Tms &tms) const
{
    const auto &def(definition_);

    const auto introId(def.introspection.tms.empty()
                       ? Resource::Id(referenceFrameId(), systemGroup()
                                      , "tms-raster-patchwork")
                       : def.introspection.tms.front());

    CesiumConf conf;
    conf.tms = tms;

    if (auto other = otherGenerator
        (Resource::Generator::Type::tms
         , addReferenceFrame(introId, referenceFrameId())))
    {
        // we have found matching tms resource, use it as an imagery provider
        const auto otherId(introId.fullId());
        const auto &otherResource(other->resource());
        const auto resdiff(resolveRoot(resource(), otherResource));

        // boundlayer path
        const fs::path blPath
            (prependRoot(fs::path(), otherResource, resdiff)
             / "boundlayer.json");
        conf.boundLayer = blPath.string();
    };

    std::ostringstream os;
    save(conf, os);
    sink.content(os.str(), fi.sinkFileInfo());
}

} // namespace generator
