#include <new>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "utility/raise.hpp"

#include "vts-libs/storage/fstreams.hpp"
#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/tileset/config.hpp"
#include "vts-libs/vts/metatile.hpp"
#include "vts-libs/vts/csconvertor.hpp"
#include "vts-libs/vts/math.hpp"
#include "vts-libs/vts/mesh.hpp"
#include "vts-libs/vts/opencv/navtile.hpp"

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
{}

Generator::Task SurfaceBase
::generateFile_impl(const FileInfo &fileInfo, const Sink::pointer &sink) const
{
    SurfaceFileInfo fi(fileInfo, config().fileFlags);

    switch (fi.type) {
    case SurfaceFileInfo::Type::unknown:
        sink->error(utility::makeError<NotFound>("Unrecognized filename."));
        break;

    case SurfaceFileInfo::Type::file: {
        switch (fi.fileType) {
        case vts::File::config: {
            if (fi.raw) {
                sink->content(vs::fileIStream
                              (fi.fileType, filePath(vts::File::config)));
            } else {
                std::ostringstream os;
                mapConfig(os, ResourceRoot::none);
                sink->content(os.str(), fi.sinkFileInfo());
            }
            break;
        }
        case vts::File::tileIndex:
            sink->content(vs::fileIStream
                          (fi.fileType, filePath(vts::File::tileIndex)));
            break;

        default:
            sink->error(utility::makeError<InternalError>
                        ("Unsupported file"));
            break;
        }
        break;
    }

    case SurfaceFileInfo::Type::tile: {
        switch (fi.tileType) {
        case vts::TileFile::meta:
            return[=](GdalWarper &warper) {
                generateMetatile(fi.tileId, sink, fi, warper);
            };

        case vts::TileFile::mesh:
            return[=](GdalWarper &warper) {
                generateMesh(fi.tileId, sink, fi, warper);
            };

        case vts::TileFile::atlas:
            sink->error(utility::makeError<NotFound>
                        ("No internal texture present."));
            break;

        case vts::TileFile::navtile:
            return[=](GdalWarper &warper) {
                generateNavtile(fi.tileId, sink, fi, warper);
            };
            break;
        }
        break;
    }

    case SurfaceFileInfo::Type::support:
        sink->content(fi.support->data, fi.support->size
                      , fi.sinkFileInfo(), false);
        break;

    default:
        sink->error(utility::makeError<InternalError>
                    ("Not implemented yet."));
    }

    return {};
}

} // namespace generator
