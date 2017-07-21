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

#include <opencv2/highgui/highgui.hpp>

#include "utility/raise.hpp"
#include "utility/path.hpp"

#include "imgproc/rastermask/cvmat.hpp"
#include "imgproc/png.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

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
#include "vts-libs/vts/2d.hpp"
#include "vts-libs/vts/debug.hpp"
#include "vts-libs/vts/mapconfig.hpp"
#include "vts-libs/registry/json.hpp"
#include "vts-libs/registry/py.hpp"

#include "../error.hpp"
#include "../support/metatile.hpp"
#include "../support/mesh.hpp"
#include "../support/srs.hpp"
#include "../support/grid.hpp"
#include "../support/python.hpp"
#include "../support/serialization.hpp"
#include "../support/mmapped/qtree-rasterize.hpp"

#include "./surface.hpp"

namespace fs = boost::filesystem;
namespace vr = vtslibs::registry;
namespace vs = vtslibs::storage;
namespace vts = vtslibs::vts;

namespace generator {

bool SurfaceBase::Introspection::empty() const
{
    return (tms.empty() && geodata.empty() && !position);
}

bool SurfaceBase::Introspection::operator!=(const Introspection &other) const
{
    // introspection can safely change
    if (tms != other.tms) { return true; }
    if (geodata != other.geodata) { return true; }
    if (position != other.position) { return true; }

    return false;
}

void SurfaceBase::SurfaceDefinition::parse(const Json::Value &value)
{
    if (value.isMember("nominalTexelSize")) {
        nominalTexelSize = boost::in_place();
        Json::get(*nominalTexelSize, value, "nominalTexelSize");
    }

    if (value.isMember("mergeBottomLod")) {
        mergeBottomLod = boost::in_place();
        Json::get(*mergeBottomLod, value, "mergeBottomLod");
    }

    if (value.isMember("introspection")) {
        const auto &jintrospection(value["introspection"]);

        introspection.tms
            = introspectionListFrom(jintrospection, "tms");
        introspection.geodata
            = introspectionListFrom(jintrospection, "geodata");

        if (jintrospection.isMember("position")) {
            introspection.position
                = vr::positionFromJson(jintrospection["position"]);
        }
    }
}

void SurfaceBase::SurfaceDefinition::build(Json::Value &value) const
{
    if (nominalTexelSize) {
        value["nominalTexelSize"] = *nominalTexelSize;
    }
    if (mergeBottomLod) {
        value["mergeBottomLod"] = *mergeBottomLod;
    }

    if (!introspection.empty()) {
        auto &jintrospection(value["introspection"] = Json::objectValue);
        introspectionListTo(jintrospection, "tms", introspection.tms) ;
        introspectionListTo(jintrospection, "geodata", introspection.geodata);

        if (introspection.position) {
            jintrospection["position"] = vr::asJson(*introspection.position);
        }
    }
}

void SurfaceBase::SurfaceDefinition::parse(const boost::python::dict &value)
{
    if (value.has_key("nominalTexelSize")) {
        nominalTexelSize
            = boost::python::extract<double>(value["nominalTexelSize"]);
    }

    if (value.has_key("mergeBottomLod")) {
        mergeBottomLod = boost::python::extract
            <vts::Lod>(value["mergeBottomLod"]);
    }

    if (value.has_key("introspection")) {
        boost::python::dict pintrospection(value["introspection"]);
        introspection.tms
            = introspectionListFrom(pintrospection, "tms");
        introspection.geodata
            = introspectionListFrom(pintrospection, "geodata");

        if (pintrospection.has_key("position")) {
            introspection.position = boost::in_place();
            vr::fromPython(*introspection.position
                           , pintrospection["position"]);
        }
    }
}

Changed SurfaceBase::SurfaceDefinition::changed_impl(const DefinitionBase &o)
    const
{
    const auto &other(o.as<SurfaceDefinition>());

    // manually set data can be changed safely
    if (nominalTexelSize != other.nominalTexelSize) {
        return Changed::safely;
    }

    if (mergeBottomLod != other.mergeBottomLod) {
        return Changed::safely;
    }

    if (introspection != other.introspection) {
        return Changed::safely;
    }

    return Changed::no;
}

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
{}

bool SurfaceBase::loadFiles(const SurfaceDefinition &definition)
{
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

bool SurfaceBase::updateProperties(const SurfaceDefinition &def)
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

    return changed;
}

Generator::Task SurfaceBase
::generateFile_impl(const FileInfo &fileInfo, Sink &sink) const
{
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
                             , FileClass::data);
                break;

            case vts::FileFlavor::debug: {
                std::ostringstream os;
                const auto debug
                    (vts::debugConfig
                     (vts::meshTilesConfig
                      (properties_, vts::ExtraTileSetProperties()
                       , prependRoot(fs::path(), resource()
                                     , ResourceRoot::none))
                      , referenceFrameId()));
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
                         , FileClass::data);
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

    const auto raw(fi.flavor == vts::FileFlavor::raw);

    auto mesh(generateMeshImpl(nodeInfo, sink, fi, arsenal, raw));

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
    vts::Mesh mesh(true);

    if (!vts::TileIndex::Flag::isWatertight(flags)) {
        mesh = generateMeshImpl
            (nodeInfo, sink, fi, arsenal, true);
    }

    if (debug) {
        sink.content(imgproc::png::serialize
                     (vts::debugMask(mesh.coverageMask, { 1 }), 9)
                     , fi.sinkFileInfo());
    } else {
        sink.content(imgproc::png::serialize
                     (vts::mask2d(mesh.coverageMask, { 1 }), 9)
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
SurfaceBase::extraProperties(const SurfaceDefinition &def) const
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

    return extra;
}

} // namespace generator
