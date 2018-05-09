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

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/replace.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"
#include "utility/format.hpp"
#include "utility/path.hpp"

#include "geo/heightcoding.hpp"
#include "geo/srsfactors.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "../support/python.hpp"
#include "../support/serialization.hpp"

#include "vts-libs/vts/opencv/navtile.hpp"

#include "../support/python.hpp"
#include "../support/tileindex.hpp"
#include "../support/srs.hpp"
#include "../support/revision.hpp"

#include "./geodata-vector-tiled.hpp"
#include "./factory.hpp"
#include "./metatile.hpp"

namespace ba = boost::algorithm;

namespace vr = vtslibs::registry;
namespace fs = boost::filesystem;

namespace generator {

namespace {

void parseDefinition(GeodataVectorTiled::Definition &def
                     , const Json::Value &value)
{
    if (value.isMember("maxSourceLod")) {
        def.maxSourceLod = boost::in_place();
        Json::get(*def.maxSourceLod, value, "maxSourceLod");
    }
}

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<GeodataVectorTiled>(params);
    }

    virtual DefinitionBase::pointer definition() {
        return std::make_shared<GeodataVectorTiled::Definition>();
    }

private:
    static utility::PreMain register_;
};

void parseDefinition(GeodataVectorTiled::Definition &def
                     , const boost::python::dict &value)
{
    if (value.has_key("maxSourceLod")) {
        def.maxSourceLod = boost::python::extract<int>(value["maxSourceLod"]);
    }
}

void buildDefinition(Json::Value &value
                     , const GeodataVectorTiled::Definition &def)
{
    if (def.maxSourceLod) {
        value["maxSourceLod"] = *def.maxSourceLod;
    }
}

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (Resource::Generator(Resource::Generator::Type::geodata
                             , "geodata-vector-tiled")
         , std::make_shared<Factory>());
});

/** NOTICE: increment each time some data-related bug is fixed.
 */
int GeneratorRevision(0);

} // namespace

void GeodataVectorTiled::Definition::from_impl(const boost::any &value)
{
    // deserialize parent class first
    GeodataVectorBase::Definition::from_impl(value);

    if (const auto *json = boost::any_cast<Json::Value>(&value)) {
        parseDefinition(*this, *json);
    } else if (const auto *py
               = boost::any_cast<boost::python::dict>(&value))
    {
        parseDefinition(*this, *py);
    } else {
        LOGTHROW(err1, Error)
            << "GeodataVectorTiled: Unsupported configuration from: <"
            << value.type().name() << ">.";
    }
}

void GeodataVectorTiled::Definition::to_impl(boost::any &value) const
{
    // serialize parent class first
    GeodataVectorBase::Definition::to_impl(value);

    if (auto *json = boost::any_cast<Json::Value>(&value)) {
        buildDefinition(*json, *this);
    } else {
        LOGTHROW(err1, Error)
            << "GeodataVectorTiled:: Unsupported serialization into: <"
            << value.type().name() << ">.";
    }
}

Changed GeodataVectorTiled::Definition::changed_impl(const DefinitionBase &o)
    const
{
    // first check parent class for change
    const auto changed(GeodataVectorBase::Definition::changed_impl(o));
    if (changed == Changed::yes) { return changed; }

    const auto &other(o.as<Definition>());

    // max source lod leads to revision bump
    if (maxSourceLod != other.maxSourceLod) {
        return Changed::withRevisionBump;
    }

    // pass result from parent
    return changed;
}

GeodataVectorTiled::GeodataVectorTiled(const Params &params)
    : GeodataVectorBase(params, true)
    , definition_(this->resource().definition<Definition>())
    , dem_(absoluteDataset(definition_.dem.dataset + "/dem")
           , definition_.dem.geoidGrid)
    , effectiveGsdArea_(), effectiveGsdAreaComputed_(false)
    , tileFile_(definition_.dataset)
    , physicalSrs_
      (vr::system.srs(resource().referenceFrame->model.physicalSrs))
{
    {
        // open dataset and get descriptor + metadata
        auto ds(geo::GeoDataset::open(dem_.dataset));
        demDescriptor_ = ds.descriptor();

        auto md(ds.getMetadata("vts"));

        if (auto effectiveGSD = md.get<double>("effectiveGSD")) {
            effectiveGsdArea_ = (*effectiveGSD * *effectiveGSD);

            LOG(info2)
                << "<" << id() << ">: using configured effective GSD area of "
                << definition_.dem.dataset << ": "
                << effectiveGsdArea_ << " m2.";
        } else {
            auto esize(math::size(demDescriptor_.extents));
            auto srs(demDescriptor_.srs.reference());
            if (srs.IsGeographic()) {
                // geographic coordinates system -> convert degrees to meters
                double a(srs.GetSemiMajor());
                // double b(srs.GetSemiMinor());

                esize.width *= (a * M_PI / 180.0);

                // TODO: compute length of arc between extents.ll(1) nad
                // extents.ur(1) on the ellipsoid
                // for now, use same calculation as on the equator
                esize.height *= (a * M_PI / 180.0);
            }

            math::Size2f px(esize.width / demDescriptor_.size.width
                            , esize.height / demDescriptor_.size.height);

            effectiveGsdArea_ = math::area(px);
            effectiveGsdAreaComputed_ = true;

            LOG(info2)
                << id() << ": using computed effective GSD area "
                << definition_.dem.dataset << ": "
                << effectiveGsdArea_ << " m2.";
        }
    }

    try {
        auto indexPath(root() / "tileset.index");
        auto deliveryIndexPath(root() / "delivery.index");

        if (fs::exists(indexPath)) {
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
            return;
        }
    } catch (const std::exception &e) {
        // not ready
    }

    LOG(info1) << "Generator for <" << id() << "> not ready.";
}

void GeodataVectorTiled::prepare_impl(Arsenal&)
{
    LOG(info2) << "Preparing <" << id() << ">.";

    const auto &r(resource());

    // try to open datasets
    geo::GeoDataset::open(dem_.dataset);
    geo::GeoDataset::open(dem_.dataset + ".min");
    geo::GeoDataset::open(dem_.dataset + ".max");

    // prepare tile index
    {
        vts::tileset::Index index(referenceFrame().metaBinaryOrder);
        prepareTileIndex(index
                         , (absoluteDataset(definition_.dem.dataset)
                            + "/tiling." + r.id.referenceFrame)
                         , r);

        // save it all
        vts::tileset::saveTileSetIndex(index, root() / "tileset.index");

        const auto deliveryIndexPath(root() / "delivery.index");
        // convert it to delivery index (using a temporary file)
        const auto tmpPath(utility::addExtension
                           (deliveryIndexPath, ".tmp"));
        mmapped::TileIndex::write(tmpPath, index.tileIndex);
        fs::rename(tmpPath, deliveryIndexPath);

        index_ = boost::in_place(referenceFrame().metaBinaryOrder
                                 , deliveryIndexPath);
    }
}

vr::FreeLayer GeodataVectorTiled::freeLayer_impl(ResourceRoot root) const
{
    const auto &res(resource());

    vr::FreeLayer fl;
    fl.id = res.id.fullId();
    fl.type = vr::FreeLayer::Type::geodataTiles;

    auto &def(fl.createDefinition<vr::FreeLayer::GeodataTiles>());
    def.metaUrl = prependRoot
        (utility::format("{lod}-{x}-{y}.meta?gr=%d-%d%s", GeneratorRevision
                         , vts::MetaTile::currentVersion()
                         , RevisionWrapper(res.revision, "&"))
         , resource(), root);
    def.geodataUrl = prependRoot
        (utility::format("{lod}-{x}-{y}.geo?gr=%d%s&viewspec={viewspec}"
                         , GeneratorRevision
                         , RevisionWrapper(res.revision, "&"))
         , resource(), root);
    def.style = styleUrl();

    def.lodRange = res.lodRange;
    def.tileRange = res.tileRange;
    fl.credits = asInlineCredits(res);

    // done
    return fl;
}

vts::MapConfig GeodataVectorTiled::mapConfig_impl(ResourceRoot root)
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
        LOG(info1) << "trying to find surface";
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

void GeodataVectorTiled::generateMetatile(Sink &sink
                                          , const GeodataFileInfo &fi
                                          , Arsenal &arsenal) const
{
    sink.checkAborted();

    if (!index_->meta(fi.tileId)) {
        sink.error(utility::makeError<NotFound>("Metatile not found."));
        return;
    }

    auto metatile(metatileFromDem
                  (fi.tileId, sink, arsenal, resource()
                   , index_->tileIndex, dem_.dataset
                   , dem_.geoidGrid
                   , MaskTree(), definition_.displaySize));

    // write metatile to stream
    std::ostringstream os;
    metatile.save(os);
    sink.content(os.str(), fi.sinkFileInfo());
}

void GeodataVectorTiled::generateGeodata(Sink &sink
                                         , const GeodataFileInfo &fi
                                         , Arsenal &arsenal) const
{
    const auto &tileId(fi.tileId);
    auto flags(index_->tileIndex.get(tileId));
    if (!vts::TileIndex::Flag::isReal(flags)) {
        sink.error(utility::makeError<NotFound>("No geodata for this tile."));
        return;
    }

    vts::NodeInfo nodeInfo(referenceFrame(), tileId);
    if (!nodeInfo.productive()) {
        sink.error(utility::makeError<NotFound>
                   ("TileId outside of valid reference frame tree."));
        return;
    }

    auto sourceTileId(tileId);
    auto sourceLocalId(vts::local(nodeInfo.rootLod(), sourceTileId));
    auto tileExtents(nodeInfo.extents());

    bool cutting(false);

    if (definition_.maxSourceLod && (sourceLocalId.lod
                                     > *definition_.maxSourceLod))
    {
        // shift source to proper layer
        auto lodDiff(sourceLocalId.lod - *definition_.maxSourceLod);
        sourceTileId = vts::parent(sourceTileId, lodDiff);
        sourceLocalId = vts::parent(sourceLocalId, lodDiff);
        tileExtents = vts::NodeInfo(referenceFrame(), sourceTileId).extents();
        cutting = true;
    }

    const auto tileFile
        (absoluteDataset
         (tileFile_(vts::UrlTemplate::Vars(sourceTileId, sourceLocalId))));

    LOG(info1) << "Using geo file: <" << tileFile << ">.";

    // combine all dem datasets and default/fallback dem dataset
    auto datasets(viewspec2datasets(fi.fileInfo.query, dem_));

    geo::heightcoding::Config config;
    config.workingSrs = sds(nodeInfo, dem_.geoidGrid);
    config.outputSrs = boost::in_place
        (physicalSrs_.srsDef, physicalSrs_.adjustVertical());
    config.layers = definition_.layers;
    config.format = definition_.format;
    config.formatConfig = definition_.formatConfig;
    config.mode = definition_.mode;

    if (cutting) {
        // clip whole tile to node extents
        config.clipWorkingExtents = nodeInfo.extents();
    } else if (definition_.clipLayers) {
        // clip only given layers
        config.clipWorkingExtents = nodeInfo.extents();
        config.clipLayers = definition_.clipLayers;
    }

    // build open options for MVT driver
    GdalWarper::OpenOptions openOptions;
    {
        std::ostringstream os;
        os << "@MVT_EXTENTS=" << std::fixed << tileExtents;
        openOptions.push_back(os.str());

        os.str("");
        os << "@MVT_SRS=" << *config.workingSrs;
        openOptions.push_back(os.str());
    }

    // heightcode data using warper's machinery
    auto hc(arsenal.warper.heightcode
            (tileFile, datasets.first, config, dem_.geoidGrid
             , openOptions, layerEnhancers(), sink));

    // force 1 hour max age if not all views from viewspec have been found
    boost::optional<long> maxAge;
    if (!datasets.second) { maxAge = 3600; }

    sink.content(hc->data, hc->size
                 , fi.sinkFileInfo().setMaxAge(maxAge), true);
}

} // namespace generator
