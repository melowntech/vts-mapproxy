#include "utility/premain.hpp"

#include "./geodata-vector-tiled.hpp"
#include "./factory.hpp"

namespace vr = vadstena::registry;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Config &config
                                      , const Resource &resource)
    {
        return std::make_shared<GeodataVectorTiled>(config, resource);
    }

    virtual DefinitionBase::pointer definition() {
        return std::make_shared<GeodataVectorBase::Definition>();
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (Resource::Generator(Resource::Generator::Type::geodata
                             , "geodata-vector-tiled")
         , std::make_shared<Factory>());
});

} // namespace

GeodataVectorTiled::GeodataVectorTiled(const Config &config
                                       , const Resource &resource)
    : GeodataVectorBase(config, resource)
{
    LOG(info1) << "Generator for <" << resource.id << "> not ready.";
}

void GeodataVectorTiled::prepare_impl()
{
    LOG(info2) << "Preparing <" << resource().id << ">.";

    // try to open datasets
    auto dataset(geo::GeoDataset::open
                 (absoluteDataset(definition_.demDataset)));

    makeReady();
}

vr::FreeLayer GeodataVectorTiled::freeLayer(ResourceRoot root) const
{
    const auto &res(resource());

    vr::FreeLayer fl;
    fl.id = res.id.fullId();

    // TODO: implement me
    (void) root;

    // done
    return fl;
}

vts::MapConfig GeodataVectorTiled::mapConfig_impl(ResourceRoot root)
    const
{
    const auto &res(resource());

    vts::MapConfig mapConfig;
    mapConfig.referenceFrame = *res.referenceFrame;

    (void) root;
    // TODO: add freelayer

    // // this is Tiled service: we have bound layer only; use remote definition
    // mapConfig.boundLayers.add
    //     (vr::BoundLayer
    //      (res.id.fullId()
    //       , prependRoot(std::string("boundlayer.json"), resource(), root)));

    return mapConfig;
}

Generator::Task GeodataVectorTiled::generateFile_impl(const FileInfo &fileInfo
                                                      , Sink &sink) const
{
    (void) fileInfo;
    (void) sink;

    // parse input filename, use tiled version
    GeodataFileInfo fi(fileInfo, true);

#if 0
    TmsFileInfo fi(fileInfo);

    // check for valid tileId
    switch (fi.type) {
    case TmsFileInfo::Type::image:
    case TmsFileInfo::Type::mask:
        if (!checkRanges(resource(), fi.tileId)) {
            sink.error(utility::makeError<NotFound>
                        ("TileId outside of configured range."));
            return {};
        }
        break;

    case TmsFileInfo::Type::metatile:
        if (!hasMetatiles_) {
            sink.error(utility::makeError<NotFound>
                        ("This dataset doesn't provide metatiles."));
            return {};
        }
        if (!checkRanges(resource(), fi.tileId, RangeType::lod)) {
            sink.error(utility::makeError<NotFound>
                        ("TileId outside of configured range."));
            return {};
        }
        break;

    default: break;
    }

    switch (fi.type) {
    case TmsFileInfo::Type::unknown:
        sink.error(utility::makeError<NotFound>("Unrecognized filename."));
        break;

    case TmsFileInfo::Type::config: {
        std::ostringstream os;
        mapConfig(os, ResourceRoot::none);
        sink.content(os.str(), fi.sinkFileInfo());
    };

    case TmsFileInfo::Type::definition: {
        std::ostringstream os;
        vr::saveBoundLayer(os, boundLayer(ResourceRoot::none));
        sink.content(os.str(), fi.sinkFileInfo());
        break;
    }

    case TmsFileInfo::Type::support:
        sink.content(fi.support->data, fi.support->size
                      , fi.sinkFileInfo(), false);
        break;

    case TmsFileInfo::Type::image: {
        if (fi.format != definition_.format) {
            sink.error(utility::makeError<NotFound>
                        ("Format <%s> is not supported by this resource (%s)."
                         , fi.format, definition_.format));
            return {};
        }

        return[=](Sink &sink, Arsenal &arsenal) {
            generateTileImage(fi.tileId, sink, arsenal);
        };
    }

    case TmsFileInfo::Type::mask:
        return [=](Sink &sink, Arsenal &arsenal)  {
            generateTileMask(fi.tileId, sink, arsenal);
        };

    case TmsFileInfo::Type::metatile:
        return [=](Sink &sink, Arsenal &arsenal) {
            generateMetatile(fi.tileId, sink, arsenal);
        };
    }
#endif

    return {};
}

} // namespace generator
