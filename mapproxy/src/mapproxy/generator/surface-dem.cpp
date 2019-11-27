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

#include "../error.hpp"
#include "../support/metatile.hpp"
#include "../support/mesh.hpp"
#include "../support/srs.hpp"
#include "../support/geo.hpp"
#include "../support/grid.hpp"
#include "../support/coverage.hpp"
#include "../support/tileindex.hpp"

#include "surface-dem.hpp"
#include "factory.hpp"
#include "metatile.hpp"

namespace fs = boost::filesystem;
namespace vr = vtslibs::registry;
namespace vs = vtslibs::storage;
namespace vts = vtslibs::vts;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<SurfaceDem>(params);
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType<SurfaceDem>(std::make_shared<Factory>());
});

} // namespace

SurfaceDem::SurfaceDem(const Params &params)
    : SurfaceBase(params)
    , definition_(resource().definition<Definition>())
    , dem_(absoluteDataset(definition_.dem.dataset + "/dem")
           , definition_.dem.geoidGrid)
    , maskTree_(absoluteDatasetRf(definition_.mask))
{
    if (loadFiles(definition_)) {
        // remember dem in registry
        addToRegistry();
    }
}

SurfaceDem::~SurfaceDem()
{
    removeFromRegistry();
}

void SurfaceDem::prepare_impl(Arsenal&)
{
    LOG(info2) << "Preparing <" << id() << ">.";

    const auto &r(resource());

    // try to open datasets
    auto dataset(geo::GeoDataset::open(dem_.dataset));
    auto datasetMin(geo::GeoDataset::open(dem_.dataset + ".min"));
    auto datasetMax(geo::GeoDataset::open(dem_.dataset + ".max"));

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
    properties_.revision = r.revision;

    // optional tuning properties
    properties_.nominalTexelSize = definition_.nominalTexelSize;
    if (definition_.mergeBottomLod) {
        properties_.mergeBottomLod = *definition_.mergeBottomLod;
    }

    {
        vts::tileset::Index index(referenceFrame().metaBinaryOrder);
        prepareTileIndex(index
                         , (absoluteDataset(definition_.dem.dataset)
                            + "/tiling." + r.id.referenceFrame)
                         , r, true, maskTree_);

        // save it all
        vts::tileset::saveConfig(filePath(vts::File::config), properties_);
        vts::tileset::saveTileSetIndex(index, filePath(vts::File::tileIndex));

        const auto deliveryIndexPath(root() / "delivery.index");
        // convert it to delivery index (using a temporary file)
        const auto tmpPath(utility::addExtension
                           (deliveryIndexPath, ".tmp"));
        mmapped::TileIndex::write(tmpPath, index.tileIndex);
        fs::rename(tmpPath, deliveryIndexPath);

        // open delivery index
        index_ = boost::in_place(referenceFrame().metaBinaryOrder
                                 , deliveryIndexPath);
    }

    addToRegistry();
}

void SurfaceDem::addToRegistry()
{
    demRegistry().add(DemRegistry::Record
                      (DemRegistry::Id(referenceFrameId(), id().fullId())
                       , dem_, id()));
    if (definition_.heightcodingAlias) {
        demRegistry().add(DemRegistry::Record
                          (DemRegistry::Id(referenceFrameId()
                                           , *definition_.heightcodingAlias)
                           , dem_, id()));
    }
}

void SurfaceDem::removeFromRegistry()
{
    demRegistry().remove(DemRegistry::Id(referenceFrameId(), id().fullId()));
    if (definition_.heightcodingAlias) {
        demRegistry().remove
            (DemRegistry::Id(referenceFrameId()
                             , *definition_.heightcodingAlias));
    }
}

vts::MapConfig SurfaceDem::mapConfig_impl(ResourceRoot root) const
{
    const auto path(prependRoot(fs::path(), resource(), root));

    auto mc(vts::mapConfig
            (properties_, resource().registry, extraProperties(definition_)
             , path));

    // force 2d interface existence
    mc.surfaces.front().has2dInterface = true;

    if (!definition_.introspection.position) {
        // no introspection position, generate some

        // look down
        mc.position.orientation = { 0.0, -90.0, 0.0 };

        // take Y size of reference frame's 3D extents extents
        mc.position.verticalExtent
            = math::size(referenceFrame().division.extents).height;

        mc.position.verticalFov = config().defaultFov;
    }

    // add local services
    vts::service::addLocal(mc, path);

    return mc;
}

void SurfaceDem::generateMetatile(const vts::TileId &tileId
                                  , Sink &sink
                                  , const SurfaceFileInfo &fi
                                  , Arsenal &arsenal
                                  , vts::SubMesh::TextureMode textureMode)
    const
{
    sink.checkAborted();

    if (!index_->meta(tileId)) {
        sink.error(utility::makeError<NotFound>("Metatile not found."));
        return;
    }

    auto metatile(generateMetatileImpl(tileId, sink, arsenal, textureMode));

    // write metatile to stream
    std::ostringstream os;
    metatile.save(os);
    sink.content(os.str(), fi.sinkFileInfo());
}

vts::MetaTile
SurfaceDem::generateMetatileImpl(const vts::TileId &tileId
                                 , Sink &sink, Arsenal &arsenal
                                 , vts::SubMesh::TextureMode textureMode) const
{
    return metatileFromDem(tileId, sink, arsenal, resource()
                           , index_->tileIndex, dem_.dataset
                           , dem_.geoidGrid, maskTree_, boost::none
                           , definition_.heightFunction
                           , textureMode);
}

namespace {

inline bool validSample(double value)
{
    return (value >= -1e6);
}

class DemSampler {
public:
    /** Samples point in dem. Dilates by one pixel if pixel is invalid.
     *
     *  Pixels masked by external mask are not valid (i.e. the mask must be
     *  dilated by 1 pixel beforehand)
     */
    DemSampler(const cv::Mat &dem, const vts::NodeInfo::CoverageMask &mask
               , const HeightFunction::pointer &heightFunction)
        : dem_(dem), mask_(mask), heightFunction_(heightFunction)
    {}

    bool operator()(int i, int j, double &h) const {
        // ignore masked-out pixels
        if (!mask_.get(i, j)) { return false; }

        h = dem_.at<double>(j, i);
        if (validSample(h)) {
            if (heightFunction_) { h = (*heightFunction_)(h); }
            return true;
        }

        h = 0;
        int count(0);

        for (int jj(-1); jj <= +1; ++jj) {
            for (int ii(-1); ii <= +1; ++ii) {
                // ignore current point
                if (!(ii || jj)) { continue; }

                auto x(i + ii), y(j + jj);
                // check bounds
                if ((x < 0) || (x >= dem_.cols)
                    || (y < 0) || (y >= dem_.rows))
                    { continue; }

                // ingore masked-out pixels
                if (!mask_.get(x, y)) { continue; }

                // sample pixel and use if valid
                auto v(dem_.at<double>(y, x));
                if (!validSample(v)) { continue; }

                h += v;
                ++count;
            }
        }

        if (!count) { return false; }
        if (heightFunction_) {
            h = (*heightFunction_)(h / count);
        } else {
            h = (h / count);
        }

        return true;
    }

private:
    cv::Mat dem_;
    const vts::NodeInfo::CoverageMask &mask_;
    const HeightFunction::pointer &heightFunction_;
};

} // namespace

AugmentedMesh SurfaceDem
::generateMeshImpl(const vts::NodeInfo &nodeInfo, Sink &sink
                   , Arsenal &arsenal, const OptHeight &defaultHeight) const
{
    const int samplesPerSide(128);
    const TileFacesCalculator tileFacesCalculator;

    sink.checkAborted();

    /** warp input dataset as a DEM, with optimized size
     */
    auto dem(arsenal.warper.warp
             (GdalWarper::RasterRequest
              (GdalWarper::RasterRequest::Operation::demOptimal
               , dem_.dataset
               , nodeInfo.srsDef(), nodeInfo.extents()
               , math::Size2(samplesPerSide, samplesPerSide))
              .setNodata(defaultHeight)
              , sink));

    sink.checkAborted();

    // grab size of computed matrix, minus one to get number of edges
    math::Size2 size(dem->cols - 1, dem->rows - 1);

    // generate coverage
    auto coverage(generateCoverage(dem->cols - 1, nodeInfo, maskTree_
                                   , vts::NodeInfo::CoverageType::grid));

    DemSampler ds(*dem, coverage, definition_.heightFunction);

    // generate mesh
    auto mesh(meshFromNode(nodeInfo, size
                               , [&](int i, int j, double &h) -> bool
    {
        return ds(i, j, h);
    }));
    mesh.textureLayerId = definition_.textureLayerId;
    mesh.geoidGrid = dem_.geoidGrid;

    // simplify
    simplifyMesh(mesh.mesh, nodeInfo, tileFacesCalculator, dem_.geoidGrid);

    // done for now
    return mesh;
}

void SurfaceDem::generateNavtile(const vts::TileId &tileId
                                 , Sink &sink
                                 , const SurfaceFileInfo &fi
                                 , Arsenal &arsenal) const
{
    sink.checkAborted();

    const auto &rf(referenceFrame());

    if (!index_->navtile(tileId)) {
        sink.error(utility::makeError<NotFound>("No navtile for this tile."));
        return;
    }

    vts::NodeInfo node(rf, tileId);
    if (!node.productive()) {
        sink.error(utility::makeError<NotFound>
                    ("TileId outside of valid reference frame tree."));
        return;
    }

    auto metaId(tileId);
    {
        metaId.x &= ~((1 << rf.metaBinaryOrder) - 1);
        metaId.y &= ~((1 << rf.metaBinaryOrder) - 1);
    }

    // suboptimal solution: generate metatile
    auto metatile(generateMetatileImpl(metaId, sink, arsenal));

    const auto &extents(node.extents());
    const auto ts(math::size(extents));

    // sds -> navigation SRS convertor
    auto navConv(sds2nav(node, dem_.geoidGrid));

    sink.checkAborted();

    const auto *metanode(metatile.get(tileId, std::nothrow));
    if (!metanode) {
        sink.error(utility::makeError<NotFound>("Metatile not found."));
    }

    const auto heightRange(metanode->heightRange);

    vts::opencv::NavTile nt;
    auto ntd(nt.data());

    // generate coverage mask in grid coordinates
    auto &coverage(nt.coverageMask()
                   = generateCoverage(ntd.cols - 1, node, maskTree_
                                      , vts::NodeInfo::CoverageType::grid));

    // warp input dataset as a DEM
    auto dem(arsenal.warper.warp
             (GdalWarper::RasterRequest
              (GdalWarper::RasterRequest::Operation::dem
               , dem_.dataset
               , node.srsDef(), node.extents()
               , math::Size2(ntd.cols - 1, ntd.rows -1))
              , sink));

    sink.checkAborted();

    // set height range
    nt.heightRange(vts::NavTile::HeightRange
                   (std::floor(heightRange.min), std::ceil(heightRange.max)));
    DemSampler ds(*dem, coverage, definition_.heightFunction);

    // calculate navtile values
    math::Size2f npx(ts.width / (ntd.cols - 1)
                     , ts.height / (ntd.rows - 1));
    double h;
    for (int j(0); j < ntd.rows; ++j) {
        auto y(extents.ur(1) - j * npx.height);
        for (int i(0); i < ntd.cols; ++i) {
            // mask with node's mask
            if (!coverage.get(i, j)) { continue; }

            // try to get sample
            if (!ds(i, j, h)) {
                // masked in dem -> remove from coverage
                coverage.set(i, j, false);
                continue;
            }
            auto z(navConv
                   (math::Point3
                    (extents.ll(0) + i * npx.width, y, h))(2));
            // write
            ntd.at<vts::opencv::NavTile::DataType>(j, i) = z;
        }
    }

    // done
    std::ostringstream os;
    if (fi.flavor == vts::FileFlavor::raw) {
        // raw navtile -> serialize to on-disk format
        nt.serialize(os);
    } else {
        // just navtile itself
        nt.serializeNavtileProper(os);
    }

    sink.content(os.str(), fi.sinkFileInfo());
}

} // namespace generator
