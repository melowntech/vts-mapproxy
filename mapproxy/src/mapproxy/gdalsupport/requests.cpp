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

#include <boost/utility/in_place_factory.hpp>

#include "requests.hpp"
#include "custom.hpp"

ShRaster::ShRaster(const GdalWarper::RasterRequest &other
                   , ManagedBuffer &sm, ShRequestBase *owner)
    : sm_(sm), owner_(owner)
    , operation_(other.operation)
    , dataset_(other.dataset.data()
               , other.dataset.size()
               , sm.get_allocator<char>())
    , srs_(other.srs.srs.data()
           , other.srs.srs.size()
           , sm.get_allocator<char>())
    , srsType_(other.srs.type)
    , extents_(other.extents)
    , size_(other.size)
    , resampling_(other.resampling)
    , mask_(sm.get_allocator<char>())
    , nodata_(other.nodata)
    , response_()
{
    if (other.mask) {
        mask_.assign(other.mask->data(), other.mask->size());
    }
}

ShRaster::~ShRaster() {
    if (response_) { sm_.deallocate(response_); }
}

ShRaster::operator GdalWarper::RasterRequest() const {
    return GdalWarper::RasterRequest
        (operation_
         , std::string(dataset_.data(), dataset_.size())
         , geo::SrsDefinition(asString(srs_), srsType_)
         , extents_, size_, resampling_
         , asOptional(mask_)).setNodata(nodata_);
}

cv::Mat* ShRaster::response() {
    auto response(response_);
    response_ = 0;
    return response;
}


void ShRaster::response(bi::interprocess_mutex &mutex, cv::Mat *response)
{
    Lock lock(mutex);
    if (response_) { return; }
    response_ = response;
    owner_->done();
}

namespace {

void copyLayers(boost::optional<StringVector> &dst
                , const boost::optional<geo::heightcoding
                ::Config::LayerNames> &src
                , ManagedBuffer &sm)
{
    if (!src) { return; }

    dst = boost::in_place(sm.get_allocator<String>());
    for (const auto &str : *src) {
        dst->push_back(String(str.data(), str.size()
                              , sm.get_allocator<char>()));
    }
}

void copyLayers(boost::optional<geo::heightcoding::Config::LayerNames> &dst
                , const boost::optional<StringVector> &src)
{
    if (!src) { return; }

    dst = boost::in_place();
    for (const auto &str : *src) {
        dst->emplace_back(str.data(), str.size());
    }
}

} // namespace

ShHeightCodeConfig
::ShHeightCodeConfig(const geo::heightcoding::Config &config
                     , ManagedBuffer &sm)
    : workingSrs_(sm.get_allocator<char>())
    , workingSrsType_()

    , outputSrs_(sm.get_allocator<char>())
    , outputSrsType_()
    , outputAdjustVertical_()

    , clipWorkingExtents_(config.clipWorkingExtents)
    , format_(config.format)
    , formatConfig_(config.formatConfig)
    , mode_(config.mode)
{
    if (config.workingSrs) {
        workingSrs_.assign(config.workingSrs->srs.data()
                           , config.workingSrs->srs.size());
        workingSrsType_ = config.workingSrs->type;
    }

    if (config.outputSrs) {
        outputSrs_.assign(config.outputSrs->srs.srs.data()
                          , config.outputSrs->srs.srs.size());
        outputSrsType_ = config.outputSrs->srs.type;
        outputAdjustVertical_ = config.outputSrs->adjustVertical;
    }

    copyLayers(layers_, config.layers, sm);
    copyLayers(clipLayers_, config.clipLayers, sm);
}

ShHeightCodeConfig::operator geo::heightcoding::Config() const
{
    geo::heightcoding::Config config;
    config.workingSrs = asOptional(workingSrs_, workingSrsType_);
    if (!outputSrs_.empty()) {
        config.outputSrs
            = boost::in_place
            (geo::SrsDefinition(asString(outputSrs_), outputSrsType_)
             , outputAdjustVertical_);
    }

    config.clipWorkingExtents = clipWorkingExtents_;

    copyLayers(config.layers, layers_);
    copyLayers(config.clipLayers, clipLayers_);

    config.format = format_;
    config.formatConfig = formatConfig_;
    config.mode = mode_;

    return config;
}


ShDemDataset::ShDemDataset(const DemDataset &demDataset
                           , ManagedBuffer &sm)
    : dataset(demDataset.dataset.data(), demDataset.dataset.size()
              , sm.get_allocator<char>())
    , geoidGrid(sm.get_allocator<char>())
{
    if (demDataset.geoidGrid) {
        geoidGrid.assign(demDataset.geoidGrid->data()
                         , demDataset.geoidGrid->size());
    }
}

DemDataset ShDemDataset::demDataset() const
{
    return { asString(dataset), asOptional(geoidGrid) };
}

ShHeightCode
::ShHeightCode(const std::string &vectorDs
               , const DemDataset::list &rasterDs
               , const geo::heightcoding::Config &config
               , const boost::optional<std::string> &vectorGeoidGrid
               , const std::vector<std::string> &openOptions
               , const LayerEnhancer::map &layerEnhancers
               , ManagedBuffer &sm, ShRequestBase *owner)
    : sm_(sm), owner_(owner)
    , vectorDs_(vectorDs.data(), vectorDs.size()
                , sm.get_allocator<char>())
    , rasterDs_(sm.get_allocator<ShDemDataset>())
    , config_(config, sm)
    , vectorGeoidGrid_(sm.get_allocator<char>())
    , layerEnhancers_(sm.get_allocator<char>())
    , response_()
{
    // copy strings to shared memory
    for (const auto &dataset : rasterDs) {
        rasterDs_.push_back(ShDemDataset(dataset, sm));
    }

    if (vectorGeoidGrid) {
        vectorGeoidGrid_.assign(vectorGeoidGrid->data()
                                , vectorGeoidGrid->size());
    }

    if (!openOptions.empty()) {
        openOptions_ = boost::in_place(sm.get_allocator<String>());

        for (const auto &str : openOptions) {
            openOptions_->push_back(String(str.data(), str.size()
                                           , sm.get_allocator<char>()));
        }
    }

    // serialize layer enhancers: 4 strings per element
    for (const auto &item : layerEnhancers) {
        layerEnhancers_.emplace_back
            (item.first.data(), item.first.size()
             , sm.get_allocator<char>());
        layerEnhancers_.emplace_back
            (item.second.key.data(), item.second.key.size()
             , sm.get_allocator<char>());
        layerEnhancers_.emplace_back
            (item.second.databasePath.data(), item.second.databasePath.size()
             , sm.get_allocator<char>());
        layerEnhancers_.emplace_back
            (item.second.table.data(), item.second.table.size()
             , sm.get_allocator<char>());
    }
}

ShHeightCode::~ShHeightCode() {
    if (response_) { sm_.deallocate(response_); }
}

std::string ShHeightCode::vectorDs() const
{
    return asString(vectorDs_);
}

DemDataset::list ShHeightCode::rasterDs() const
{
    DemDataset::list ds;
    for (const auto &dataset : rasterDs_) {
        ds.push_back(dataset.demDataset());
    }
    return ds;
}

geo::heightcoding::Config ShHeightCode::config() const {
    return config_;
}

boost::optional<std::string> ShHeightCode::vectorGeoidGrid() const
{
    if (vectorGeoidGrid_.empty()) { return boost::none; }
    return std::string(vectorGeoidGrid_.data(), vectorGeoidGrid_.size());
}

std::vector<std::string> ShHeightCode::openOptions() const
{
    if (!openOptions_) { return {}; }

    std::vector<std::string> openOptions;
    for (const auto &str : *openOptions_) {
        openOptions.emplace_back(str.data(), str.size());
    }
    return openOptions;
}

LayerEnhancer::map ShHeightCode::layerEnhancers() const
{
    LayerEnhancer::map layerEnhancers;

    auto ilayerEnhancers(layerEnhancers_.begin());

    const auto nextString([&]() -> std::string
    {
        const auto &src(*ilayerEnhancers++);
        return std::string(src.data(), src.size());
    });

    for (auto elayerEnhancers(layerEnhancers_.end());
         ilayerEnhancers != elayerEnhancers; )
    {
        auto layer(nextString());
        auto key(nextString());
        auto db(nextString());
        auto table(nextString());
        layerEnhancers.insert
            (LayerEnhancer::map::value_type
             (std::move(layer)
              , LayerEnhancer
              (std::move(key), std::move(db), std::move(table))));
    }

    return layerEnhancers;
}

/** Steals response.
 */
GdalWarper::Heightcoded* ShHeightCode::response() {
    auto response(response_);
    response_ = 0;
    return response;
}

void ShHeightCode::response(bi::interprocess_mutex &mutex
                            , GdalWarper::Heightcoded *response)
{
    Lock lock(mutex);
    if (response_) { return; }
    response_ = response;
    owner_->done();
}

CustomRequest::~CustomRequest() {}
