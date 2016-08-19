#include <boost/filesystem.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"
#include "utility/path.hpp"

#include "geo/heightcoding.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/storage/fstreams.hpp"

#include "../support/python.hpp"
#include "../support/tileindex.hpp"
#include "../support/srs.hpp"

#include "./geodata-vector.hpp"
#include "./factory.hpp"

namespace vr = vadstena::registry;
namespace fs = boost::filesystem;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<GeodataVector>(params);
    }

    virtual DefinitionBase::pointer definition() {
        return std::make_shared<GeodataVector::Definition>();
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (Resource::Generator(Resource::Generator::Type::geodata
                             , "geodata-vector")
         , std::make_shared<Factory>());
});

} // namespace

GeodataVector::GeodataVector(const Params &params)
    : GeodataVectorBase(params, false)
    , definition_(this->resource().definition<Definition>())
    , dataPath_(root() / "geodata")
{
    try {
        metadata_ = geo::heightcoding::loadMetadata(root() / "metadata.json");
        if (fs::file_size(dataPath_) == metadata_.fileSize) {
            // valid file
            makeReady();
            return;
        }
        LOG(info1) << "Sizes differ, regenerate.";
    } catch (const std::exception &e) {
        // not ready
    }
    LOG(info1) << "Generator for <" << id() << "> not ready.";
}

void GeodataVector::prepare_impl(Arsenal &arsenal)
{
    // TODO: heightcode input
    LOG(info4) << "Preparing data";

    const auto &physicalSrs
        (vr::system.srs(resource().referenceFrame->model.physicalSrs));

    // height code dataset
    geo::heightcoding::Config config;
    config.outputSrs = physicalSrs.srsDef;
    config.outputVerticalAdjust = physicalSrs.adjustVertical();
    config.layers = definition_.layers;
    config.format = definition_.format;

    // heightcode data using warper's machinery
    Aborter dummyAborter;
    auto hc(arsenal.warper.heightcode
            (absoluteDataset(definition_.dataset)
             , absoluteDataset(definition_.demDataset + "/dem")
             , config, definition_.geoidGrid
             , dummyAborter));

    // save output to file
    utility::write(dataPath_, hc->data, hc->size);

    // store metadata
    metadata_ = hc->metadata;

    // write metadata
    geo::heightcoding::saveMetadata(root() / "metadata.json", metadata_);
}

vr::FreeLayer GeodataVector::freeLayer_impl(ResourceRoot root) const
{
    const auto &res(resource());

    vr::FreeLayer fl;
    fl.id = res.id.fullId();
    fl.type = vr::FreeLayer::Type::geodata;

    auto &def(fl.createDefinition<vr::FreeLayer::Geodata>());
    def.extents = metadata_.extents;
    def.displaySize = definition_.displaySize;
    def.label = res.comment;
    def.geodata = prependRoot(std::string("geo"), res, root);
    def.style = definition_.styleUrl;

    // done
    return fl;
}

vts::MapConfig GeodataVector::mapConfig_impl(ResourceRoot root)
    const
{
    const auto &res(resource());

    vts::MapConfig mapConfig;
    mapConfig.referenceFrame = *res.referenceFrame;

    // add free layer into list of free layers
    mapConfig.freeLayers.add
        (vr::FreeLayer
         (res.id.fullId()
          , prependRoot(std::string("freelayer.json"), resource(), root)));

    // add free layer into view
    mapConfig.view.freeLayers[res.id.fullId()];

    if (definition_.introspectionSurface) {
        LOG(info4) << "trying to find surface";
        if (auto other = otherGenerator
            (Resource::Generator::Type::surface
             , addReferenceFrame(*definition_.introspectionSurface
                                 , referenceFrameId())))
        {
            mapConfig.merge(other->mapConfig
                            (resolveRoot(resource(), other->resource())));
        }
    }

    // done
    return mapConfig;
}

void GeodataVector::generateMetatile(Sink &sink, const GeodataFileInfo &
                                     , Arsenal &) const
{
    sink.error(utility::makeError<NotFound>
               ("Monolithic geodata resource has no metatiles."));
}

void GeodataVector::generateGeodata(Sink &sink
                                    , const GeodataFileInfo &fi
                                    , Arsenal&) const
{
    auto sfi(fi.sinkFileInfo());
    sink.content(vs::fileIStream(sfi.contentType.c_str(), dataPath_)
                 , Sink::FileInfo::FileClass::data);
}

} // namespace generator
