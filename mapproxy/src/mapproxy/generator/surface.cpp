#include <new>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/raise.hpp"
#include "imgproc/rastermask/cvmat.hpp"

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

        case vts::TileFile::meta2d:
            return[=](GdalWarper &warper) {
                generate2dMetatile(fi.tileId, sink, fi, warper);
            };
            break;

        case vts::TileFile::mask:
            return[=](GdalWarper &warper) {
                generate2dMask(fi.tileId, sink, fi, warper);
            };
            break;

        case vts::TileFile::ortho:
            sink->error(utility::makeError<NotFound>
                        ("No orthophoto present."));
            break;

        case vts::TileFile::credits:
            return[=](GdalWarper &warper) {
                generate2dCredits(fi.tileId, sink, fi, warper);
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

void SurfaceBase::generateMesh(const vts::TileId &tileId
                               , const Sink::pointer &sink
                               , const SurfaceFileInfo &fi
                               , GdalWarper &warper) const

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
              (nodeInfo, sink, fi, warper, fi.raw));

    // write mesh (only mesh!) to stream
    std::ostringstream os;
    if (fi.raw) {
        vts::saveMesh(os, mesh);
    } else {
        vts::saveMeshProper(os, mesh);
    }

    sink->content(os.str(), fi.sinkFileInfo());
}

namespace {

/** Renders 2d mask for plain surface with just single mesh
 */
cv::Mat render2dMask(const imgproc::quadtree::RasterMask &mask)
{
    // ensure m has proper size and type
    auto size(vts::Mask2d::size());
    cv::Mat_<std::uint8_t> m(size.height, size.width, std::uint8_t(0));

    // just one submesh
    cv::Scalar color(1);

    mask.forEachQuad([&](uint xstart, uint ystart, uint xsize
                         , uint ysize, bool)
    {
        cv::rectangle(m, cv::Rect(xstart, ystart, xsize, ysize)
                      , color, CV_FILLED, 4);
    }, imgproc::quadtree::RasterMask::Filter::white);

    // plain surface
    m(vts::Mask2d::flagRow, 0) = vts::Mask2d::Flag::submesh;
    m(vts::Mask2d::surfaceRow, 0) = 1;

    return m;
}

} // namespace

void SurfaceBase::generate2dMask(const vts::TileId &tileId
                                 , const Sink::pointer &sink
                                 , const SurfaceFileInfo &fi
                                 , GdalWarper &warper) const

{
    vts::Mesh mesh(true);

    auto flags(index_.tileIndex.get(tileId));
    if (!vts::TileIndex::Flag::isReal(flags)) {
        utility::raise<NotFound>("No mesh for this tile.");
    }

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.valid()) {
        utility::raise<NotFound>
            ("TileId outside of valid reference frame tree.");
    }

    if (!vts::TileIndex::Flag::isWatertight(flags)) {
        mesh = generateMeshImpl
            (nodeInfo, sink, fi, warper, true);
    }

    // generate mask; we have just 1 submesh -> render valid area as 1
    auto mat(render2dMask(mesh.coverageMask));

    // serialize and return
    std::vector<unsigned char> buf;
    // write as png file
    cv::imencode(".png", mat, buf
                 , { cv::IMWRITE_PNG_COMPRESSION, 9 });
    sink->content(buf, fi.sinkFileInfo());
}

} // namespace generator
