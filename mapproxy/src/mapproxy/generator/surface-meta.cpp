/**
 * Copyright (c) 2019 Melown Technologies SE
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
#include <boost/logic/tribool_io.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"
#include "utility/path.hpp"

#include "geometry/mesh.hpp"

#include "geo/coordinates.hpp"

#include "imgproc/rastermask/mappedqtree.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/storage/fstreams.hpp"
#include "vts-libs/registry/json.hpp"
#include "vts-libs/registry/py.hpp"
#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/tileset/config.hpp"
#include "vts-libs/vts/metatile.hpp"
#include "vts-libs/vts/csconvertor.hpp"
#include "vts-libs/vts/math.hpp"
#include "vts-libs/vts/mesh.hpp"
#include "vts-libs/vts/opencv/navtile.hpp"
#include "vts-libs/vts/service.hpp"
#include "vts-libs/vts/tileset/tilesetindex.hpp"

#include "../error.hpp"
#include "../support/metatile.hpp"
#include "../support/mesh.hpp"
#include "../support/srs.hpp"
#include "../support/geo.hpp"
#include "../support/grid.hpp"
#include "../support/coverage.hpp"
#include "../support/tileindex.hpp"

#include "surface-meta.hpp"
#include "factory.hpp"

namespace fs = boost::filesystem;
namespace vr = vtslibs::registry;
namespace vs = vtslibs::storage;
namespace vts = vtslibs::vts;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<SurfaceMeta>(params);
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType<SurfaceMeta>(std::make_shared<Factory>());
});

} // namespace

SurfaceMeta::SurfaceMeta(const Params &params)
    : Generator(params)
    , definition_(resource().definition<Definition>())
{}

SurfaceMeta::~SurfaceMeta() {}

void SurfaceMeta::prepare_impl(Arsenal&)
{
    LOG(info2) << "Preparing <" << id() << ">.";

    // find surface and tms generators

    surface_ = otherGenerator
        (Resource::Generator::Type::surface
         , Resource::Id(referenceFrameId(), definition_.surface), true);

    tms_ = otherGenerator
        (Resource::Generator::Type::tms
         , Resource::Id(referenceFrameId(), definition_.tms), true);

    LOG(info4) << "surface: " << surface_;
    LOG(info4) << "tms: " << tms_;

    ts_ = surface_->getProvider<VtsTilesetProvider>();
    if (!ts_) {
        LOGTHROW(err2, std::runtime_error)
            << "Driver for <"
            << Resource::Id(referenceFrameId(), definition_.surface)
            << "> doesn't provide VTS tileset support.";
    }
    LOG(info4) << "ts: " << ts_;

    atlas_ = tms_->getProvider<VtsAtlasProvider>();
    if (!atlas_) {
        LOGTHROW(err2, std::runtime_error)
            << "Driver for <"
            << Resource::Id(referenceFrameId(), definition_.surface)
            << "> doesn't provide VTS atlas support.";
    }
    LOG(info4) << "atlas: " << atlas_;
}

vts::MapConfig SurfaceMeta::mapConfig_impl(ResourceRoot root) const
{
    const auto path(prependRoot(fs::path(), resource(), root));

    auto mc(vts::mapConfig
            (ts_->properties(), resource().registry, {}, path));

    // force 2d interface existence
    mc.surfaces.front().has2dInterface = true;

    // add local services
    vts::service::addLocal(mc, path);
    return mc;
}

typedef vts::TileIndex::Flag TiFlag;

Generator::Task SurfaceMeta::tileindex(const SurfaceFileInfo &fi, Sink&)
    const
{
    const auto path(ts_->path(vts::File::tileIndex).value());

    return [=](Sink &sink, Arsenal&) {
        // rewrite original tileindex to contain atlas
        vts::tileset::Index index(referenceFrame().metaBinaryOrder);
        vts::tileset::loadTileSetIndex(index, path);

        auto &ti(index.tileIndex);

        // abuse combine-with-itself to modify flags
        const auto &combiner([&](TiFlag::value_type o, TiFlag::value_type)
                             -> TiFlag::value_type
        {
            return TiFlag::isReal(o) ? (o | TiFlag::atlas) : o;
        });
        ti.combine(ti, combiner);

        std::ostringstream os;
        vts::tileset::saveTileSetIndex(index, os);
        sink.content(os.str(), fi.sinkFileInfo());
    };
}

Generator::Task SurfaceMeta
::generateFile_impl(const FileInfo &fileInfo, Sink &sink) const
{
    if (fileInfo.interface.interface != GeneratorInterface::Interface::vts) {
        return ts_->generateFile(fileInfo, sink);
    }

    SurfaceFileInfo fi(fileInfo);

    switch (fi.type) {

    case SurfaceFileInfo::Type::file: {

        switch (fi.fileType) {
        case vts::File::tileIndex:
            return tileindex(fi, sink);
        default: break;
        } break;

    } break;

    case SurfaceFileInfo::Type::tile: {

        switch (fi.tileType) {
        case vts::TileFile::meta:
            return ts_->generateMetatile(fi.tileId, sink, fi
                                         , vts::SubMesh::internal);

        case vts::TileFile::mesh:
            return ts_->generateMesh(fi.tileId, sink, fi
                                     , vts::SubMesh::internal);

        case vts::TileFile::atlas:
            return atlas_->generateAtlas
                (fi.tileId, sink, fi.sinkFileInfo()
                 , (fi.flavor == vts::FileFlavor::raw));

        default: break;
        } break;

    } break;

    default: break;
    }

    return ts_->generateFile(fileInfo, sink);
}

} // namespace generator
