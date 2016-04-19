#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"

#include "geo/geodataset.hpp"

#include "imgproc/rastermask/cvmat.hpp"

#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/tileset/config.hpp"

#include "../error.hpp"

#include "./surface-spheroid.hpp"
#include "./factory.hpp"

#include "browser2d/index.html.hpp"

namespace fs = boost::filesystem;
namespace vr = vadstena::registry;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Config &config
                                      , const Resource &resource)
    {
        return std::make_shared<SurfaceSpheroid>(config, resource);
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (resdef::SurfaceSpheroid::generator, std::make_shared<Factory>());
});

} // namespace

SurfaceSpheroid::SurfaceSpheroid(const Config &config
                                 , const Resource &resource)
    : Generator(config, resource)
    , definition_(this->resource().definition<resdef::SurfaceSpheroid>())
{
    try {
        auto indexPath(root() / "tileset.index");
        auto propertiesPath(root() / "tileset.conf");
        if (fs::exists(indexPath) && fs::exists(propertiesPath)) {
            // both paths exist -> ok
            vts::tileset::loadTileSetIndex(index_, indexPath);
            properties_ = vts::tileset::loadConfig(propertiesPath);
            makeReady();
            return;
        }
    } catch (const std::exception &e) {
        // not ready
    }
    LOG(info1) << "Generator for <" << resource.id << "> not ready.";
}

void SurfaceSpheroid::prepare_impl()
{
    LOG(info2) << "Preparing <" << resource().id << ">.";

    const auto &r(resource());

    // build properties
    properties_ = {};
    properties_.id = r.id.fullId();
    properties_.referenceFrame = r.referenceFrame->id;
    properties_.credits = asIntSet(r.credits);
    if (definition_.textureLayerId) {
        properties_.boundLayers.insert(definition_.textureLayerId);
    }
    // position ???
    // keep driverOptions empty -> no driver
    properties_.lodRange = r.lodRange;
    properties_.tileRange = r.tileRange;

    // TODO: spatialDivisionExtents

    // grab and reset tile index
    auto &ti(index_.tileIndex);
    ti = {};

    // build tile index
    for (vts::Lod lod(0); lod <= r.lodRange.max; ++lod) {
        // metatiles everywhere
        vts::TileIndex::Flag::value_type flags
            (vts::TileIndex::Flag::meta);
        if (in(lod, r.lodRange)) {
            // watertight tiles and navtiles everywhere
            flags |= (vts::TileIndex::Flag::mesh
                      | vts::TileIndex::Flag::watertight
                      | vts::TileIndex::Flag::navtile);
        }
        // set whole LOD to given value
        ti.set(lod, flags);
    }

    // save it all
    vts::tileset::saveConfig(root() / "tileset.conf", properties_);
    vts::tileset::saveTileSetIndex(index_, root() / "tileset.index");
}

vts::MapConfig SurfaceSpheroid::mapConfig_impl(ResourceRoot root)
    const
{
    return vts::mapConfig
        (properties_, vts::ExtraTileSetProperties()
         , prependRoot(fs::path(), resource(), root));
}

Generator::Task SurfaceSpheroid
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
            std::ostringstream os;
            mapConfig(os, ResourceRoot::none);
            sink->content(os.str(), fi.sinkFileInfo());
            break;
        }
        case vts::File::tileIndex:
            sink->error(utility::makeError<InternalError>
                        ("Not implemented yet."));
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
            return[=](GdalWarper &warper)  {
                generateMetatile(fi.tileId, sink, warper);
            };

        case vts::TileFile::mesh:
            sink->error(utility::makeError<InternalError>
                        ("No mesh generated yet."));
            break;

        case vts::TileFile::atlas:
            sink->error(utility::makeError<NotFound>
                        ("No internal texture present."));
            break;

        case vts::TileFile::navtile:
            sink->error(utility::makeError<InternalError>
                        ("No navtile generated yet."));
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

void SurfaceSpheroid::generateMetatile(const vts::TileId &tileId
                                       , const Sink::pointer &sink
                                       , GdalWarper &warper) const
{
    (void) tileId;
    (void) warper;

    sink->error(utility::makeError<InternalError>
                ("Not implemented yet."));
}

} // namespace generator
