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

#include <boost/filesystem.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"
#include "utility/format.hpp"
#include "utility/path.hpp"

#include "geo/heightcoding.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/storage/fstreams.hpp"

#include "../support/python.hpp"
#include "../support/tileindex.hpp"
#include "../support/srs.hpp"
#include "../support/revision.hpp"

#include "./geodata-vector.hpp"
#include "./factory.hpp"

namespace vr = vtslibs::registry;
namespace fs = boost::filesystem;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<GeodataVector>(params);
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType<GeodataVector>(std::make_shared<Factory>());
});

} // namespace

GeodataVector::GeodataVector(const Params &params)
    : GeodataVectorBase(params, false)
    , definition_(this->resource().definition<Definition>())
    , dem_(absoluteDataset(definition_.dem.dataset + "/dem")
           , definition_.dem.geoidGrid)
    , physicalSrs_(vr::system.srs(resource()
                                  .referenceFrame->model.physicalSrs))
    , dataPath_(root() / "geodata")
{
    // load geodata only if there is no enforced change
    if (changeEnforced()) {
        LOG(info1) << "Generator for <" << id() << "> not ready.";
        return;
    }

    try {
        metadata_ = geo::heightcoding::loadMetadata
            (root() / "metadata.json");
        if (fs::file_size(dataPath_) == metadata_.fileSize) {
            // valid file
            makeReady();
            return;
        }
        LOG(info1) << "Sizes differ, regenerate.";
    } catch (const std::exception &e) { /* not ready */ }

    LOG(info1) << "Generator for <" << id() << "> not ready.";
}

GdalWarper::Heightcoded::pointer
GeodataVector::heightcode(const DemDataset::list &datasets
                          , GdalWarper &warper, Aborter &aborter) const
{
    // height code dataset
    geo::heightcoding::Config config;
    config.outputSrs = boost::in_place
        (physicalSrs_.srsDef, physicalSrs_.adjustVertical());
    config.layers = definition_.layers;
    config.format = definition_.format;
    config.formatConfig = definition_.formatConfig;
    config.mode = definition_.mode;

    // heightcode data using warper's machinery
    auto hc(warper.heightcode
            (absoluteDataset(definition_.dataset)
             , datasets, config, dem_.geoidGrid, {}
             , layerEnhancers(), aborter));
    return hc;
}

void GeodataVector::prepare_impl(Arsenal &arsenal)
{
    Aborter dummyAborter;
    auto hc(heightcode({ dem_ }, arsenal.warper, dummyAborter));

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
    def.geodata = prependRoot
        (utility::format("geo?viewspec={viewspec}%s"
                         , RevisionWrapper(res.revision, "&"))
         , res, root);
    def.style = styleUrl();
    fl.credits = asInlineCredits(res);
    def.options = definition_.options;

    // done
    return fl;
}

vts::MapConfig GeodataVector::mapConfig_impl(ResourceRoot root)
    const
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

    // browser options (must be Json::Value!); overrides browser options from
    // surface's introspection
    if (!definition_.introspection.browserOptions.empty()) {
        mapConfig.browserOptions = definition_.introspection.browserOptions;
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
                                    , Arsenal &arsenal) const
{
    const auto datasets(viewspec2datasets(fi.fileInfo.query, dem_));

    // force 1 hour max age if not all views from viewspec have been found
    boost::optional<long> maxAge;
    if (!datasets.second) { maxAge = 3600; }

    if (datasets.first.size() > 1) {
        // valid viewspec -> use it to heightcode file
        auto hc(heightcode(datasets.first, arsenal.warper, sink));

        sink.content(hc->data, hc->size
                     , fi.sinkFileInfo().setMaxAge(maxAge), true);
        return;
    }

    // no valid viewspec, return original file
    sink.content(vs::fileIStream(fi.sinkFileInfo().contentType.c_str()
                                 , dataPath_)
                 , FileClass::data, maxAge);
}

} // namespace generator
