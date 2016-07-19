#include <new>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/raise.hpp"
#include "imgproc/rastermask/cvmat.hpp"
#include "imgproc/png.hpp"

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

#include "../error.hpp"
#include "../support/metatile.hpp"
#include "../support/mesh.hpp"
#include "../support/srs.hpp"
#include "../support/grid.hpp"

#include "./surface.hpp"

namespace fs = boost::filesystem;
namespace vr = vadstena::registry;
namespace vs = vadstena::storage;
namespace vts = vadstena::vts;

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

SurfaceBase::SurfaceBase(const Config &config
                       , const Resource &resource)
    : Generator(config, resource)
    , index_(resource.referenceFrame->metaBinaryOrder)
{}

Generator::Task SurfaceBase
::generateFile_impl(const FileInfo &fileInfo, Sink &sink) const
{
    SurfaceFileInfo fi(fileInfo, config().fileFlags);

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
            if (fi.raw) {
                sink.content(vs::fileIStream
                              (fi.fileType, filePath(vts::File::config)));
            } else {
                return[=](Sink &sink, Arsenal &arsenal) {
                    std::ostringstream os;
                    mapConfig(os, ResourceRoot::none, arsenal);
                    sink.content(os.str(), fi.sinkFileInfo());
                };
            }
            break;
        }

        case vts::File::tileIndex:
            sink.content(vs::fileIStream
                          (fi.fileType, filePath(vts::File::tileIndex)));
            break;

        default:
            sink.error(utility::makeError<InternalError>
                        ("Unsupported file"));
            break;
        }
        break;
    }

    case SurfaceFileInfo::Type::tile: {
        switch (fi.tileType) {
        case vts::TileFile::meta:
            return[=](Sink &sink, Arsenal &arsenal) {
                generateMetatile(fi.tileId, sink, fi, arsenal);
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
        sink.content(fi.support->data, fi.support->size
                      , fi.sinkFileInfo(), false);
        break;

    case SurfaceFileInfo::Type::registry:
        sink.content(vs::fileIStream
                      (fi.registry->contentType, fi.registry->path));
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
    auto flags(index_.tileIndex.get(tileId));
    if (!vts::TileIndex::Flag::isReal(flags)) {
        utility::raise<NotFound>("No mesh for this tile.");
    }

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        utility::raise<NotFound>
            ("TileId outside of valid reference frame tree.");
    }

    auto mesh(generateMeshImpl
              (nodeInfo, sink, fi, arsenal, fi.raw));

    // write mesh (only mesh!) to stream
    std::ostringstream os;
    if (fi.raw) {
        vts::saveMesh(os, mesh);
    } else {
        vts::saveMeshProper(os, mesh);
    }

    sink.content(os.str(), fi.sinkFileInfo());
}

void SurfaceBase::generate2dMask(const vts::TileId &tileId
                                 , Sink &sink
                                 , const SurfaceFileInfo &fi
                                 , Arsenal &arsenal) const

{
    auto flags(index_.tileIndex.get(tileId));
    if (!vts::TileIndex::Flag::isReal(flags)) {
        utility::raise<NotFound>("No mesh for this tile.");
    }

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        utility::raise<NotFound>
            ("TileId outside of valid reference frame tree.");
    }

    // by default full watertight mesh
    vts::Mesh mesh(true);

    if (!vts::TileIndex::Flag::isWatertight(flags)) {
        mesh = generateMeshImpl
            (nodeInfo, sink, fi, arsenal, true);
    }

    sink.content(imgproc::serialize(vts::mask2d(mesh.coverageMask, { 1 }), 9)
                  , fi.sinkFileInfo());
}

void SurfaceBase::generate2dMetatile(const vts::TileId &tileId
                                     , Sink &sink
                                     , const SurfaceFileInfo &fi
                                     , Arsenal&) const

{
    sink.content(imgproc::serialize(vts::meta2d(index_.tileIndex, tileId), 9)
                  , fi.sinkFileInfo());
}

void SurfaceBase::generateCredits(const vts::TileId&
                                  , Sink &sink
                                  , const SurfaceFileInfo &fi
                                  , Arsenal&) const
{
    vts::CreditTile creditTile;
    creditTile.credits = asInlineCredits(resource().credits);

    std::ostringstream os;
    saveCreditTile(os, creditTile, true);
    sink.content(os.str(), fi.sinkFileInfo());
}

} // namespace generator
