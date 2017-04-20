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

#include "./requests.hpp"

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
         , asOptional(mask_));
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

ShHeightCodeConfig::ShHeightCodeConfig(const geo::heightcoding::Config &config
                                       , ManagedBuffer &sm)
    : workingSrs_(sm.get_allocator<char>())
    , workingSrsType_()

    , outputSrs_(sm.get_allocator<char>())
    , outputSrsType_()

    , clipWorkingExtents_(config.clipWorkingExtents)
    , format_(config.format)
{
    if (config.workingSrs) {
        workingSrs_.assign(config.workingSrs->srs.data()
                           , config.workingSrs->srs.size());
        workingSrsType_ = config.workingSrs->type;
    }

    if (config.outputSrs) {
        outputSrs_.assign(config.outputSrs->srs.data()
                          , config.outputSrs->srs.size());
        outputSrsType_ = config.outputSrs->type;
    }

    if (config.layers) {
        layers_ = boost::in_place(sm.get_allocator<String>());

        for (const auto &str : *config.layers) {
            layers_->push_back(String(str.data(), str.size()
                                      , sm.get_allocator<char>()));
        }
    }
}

ShHeightCodeConfig::operator geo::heightcoding::Config() const
{
    geo::heightcoding::Config config;
    config.workingSrs = asOptional(workingSrs_, workingSrsType_);
    config.outputSrs = asOptional(outputSrs_, outputSrsType_);
    config.clipWorkingExtents = clipWorkingExtents_;

    if (layers_) {
        config.layers = boost::in_place();
        for (const auto &str : *layers_) {
            config.layers->emplace_back(str.data(), str.size());
        }
    }

    config.format = format_;

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
               , ManagedBuffer &sm, ShRequestBase *owner)
    : sm_(sm), owner_(owner)
    , vectorDs_(vectorDs.data(), vectorDs.size()
                , sm.get_allocator<char>())
    , rasterDs_(sm.get_allocator<ShDemDataset>())
    , config_(config, sm)
    , vectorGeoidGrid_(sm.get_allocator<char>())
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

ShNavtile::ShNavtile(const GdalWarper::Navtile &navtile, ManagedBuffer &sm)
    : path(navtile.path.data(), navtile.path.size(), sm.get_allocator<char>())
    , raw(navtile.raw.data(), navtile.raw.size(), sm.get_allocator<char>())
    , extents(navtile.extents)
    , sdsSrs(navtile.sdsSrs.data(), navtile.sdsSrs.size()
             , sm.get_allocator<char>())
    , navSrs(navtile.navSrs.data(), navtile.navSrs.size()
             , sm.get_allocator<char>())
    , heightRange(navtile.heightRange)
{}

GdalWarper::Navtile ShNavtile::navtile(bool noRaw) const
{
    GdalWarper::Navtile navtile;
    navtile.path = asString(path);
    if (!noRaw) { navtile.raw = asString(raw); }
    navtile.extents = extents;
    navtile.sdsSrs = asString(sdsSrs);
    navtile.navSrs = asString(navSrs);
    navtile.heightRange = heightRange;
    return navtile;
}

ConstBlock ShNavtile::rawData() const
{
    return { raw.data(), raw.size() };
}

ShNavHeightCode
::ShNavHeightCode(const std::string &vectorDs
                 , const GdalWarper::Navtile &navtile
                 , const geo::heightcoding::Config &config
                  , const std::string &fallbackDs
                  , const boost::optional<std::string> &geoidGrid
                 , ManagedBuffer &sm, ShRequestBase *owner)
    : sm_(sm), owner_(owner)
    , vectorDs_(vectorDs.data(), vectorDs.size()
                , sm.get_allocator<char>())
    , navtile_(navtile, sm)
    , config_(config, sm)
    , fallbackDs_(fallbackDs.data(), fallbackDs.size()
                  , sm.get_allocator<char>())
    , geoidGrid_(sm.get_allocator<char>())
    , response_()
{
    if (geoidGrid) {
        geoidGrid_.assign(geoidGrid->data(), geoidGrid->size());
    }
}

ShNavHeightCode::~ShNavHeightCode() {
    if (response_) { sm_.deallocate(response_); }
}

std::string ShNavHeightCode::vectorDs() const {
    return asString(vectorDs_);
}

GdalWarper::Navtile ShNavHeightCode::navtile(bool noRaw) const {
    return navtile_.navtile(noRaw);
}

ConstBlock ShNavHeightCode::rawData() const {
    return navtile_.rawData();
}

geo::heightcoding::Config ShNavHeightCode::config() const {
    return config_;
}

std::string ShNavHeightCode::fallbackDs() const {
    return asString(fallbackDs_);
}

boost::optional<std::string> ShNavHeightCode::geoidGrid() const {
    return asOptional(geoidGrid_);
}

/** Steals response.
 */
GdalWarper::Heightcoded* ShNavHeightCode::response() {
    auto response(response_);
    response_ = 0;
    return response;
}

void ShNavHeightCode::response(bi::interprocess_mutex &mutex
                              , GdalWarper::Heightcoded *response)
{
    Lock lock(mutex);
    if (response_) { return; }
    response_ = response;
    owner_->done();
}
