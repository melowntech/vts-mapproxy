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

#include <stdlib.h>
#include <unistd.h>

#include <cerrno>
#include <fstream>
#include <system_error>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"
#include "utility/format.hpp"
#include "utility/path.hpp"

#include "imgproc/rastermask/cvmat.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "../error.hpp"

#include "factory.hpp"
#include "tms-windyty.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace bio = boost::iostreams;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<TmsWindyty>(params);
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType<TmsWindyty>(std::make_shared<Factory>());
});

// definition manipulation

// definition dataset config

TmsWindyty::DatasetConfig loadConfig(const fs::path &path)
{
    LOG(info2) << "Loading windyty config from " << path << ".";
    TmsWindyty::DatasetConfig config;

    po::options_description o;
    o.add_options()
        ("windyty.base", po::value(&config.base)->required()
         , "Base time (in second since epoch).")
        ("windyty.period", po::value(&config.period)->required()
         , "Forcast period (in seconds).")
        ("windyty.urlTemplate", po::value(&config.urlTemplate)->required()
         , "URL template.")
        ("windyty.srs", po::value(&config.srs)->required()
         , "Dataset SRS (EPSG id).")
        ("windyty.extents", po::value(&config.extents)->required()
         , "Dataset extents (in dataset SRS).")
        ("windyty.maxLod", po::value(&config.maxLod)->required()
         , "Maximum tiling level of detail.")
        ("windyty.overviews", po::value(&config.overviews)->required()
         , "Number of overviews (levels of detail) to advertise.")
        ("windyty.transparent", po::value(&config.transparent)
         ->default_value(config.transparent)->required()
         , "Images are transparent.")
        ;

    std::ifstream f;
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try {
        po::variables_map vm;
        f.open(path.c_str());
        f.exceptions(std::ifstream::badbit);
        po::store(po::parse_config_file(f, o), vm);
        po::notify(vm);
        f.close();
    } catch(const std::ios_base::failure &e) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot read windyty dataset file " << path << ": <"
            << e.what() << ">.";
    }

    return config;
}

std::time_t normalizedTime(std::time_t time,
                           const TmsWindyty::DatasetConfig &config)
{
    // time difference
    auto diff(time - config.base);

    // subtract partial period
    auto fixed(time - (diff % config.period));

    // // move to next period if inside
    // if (fixed < time) { fixed += config.period; }

    return fixed;
}

std::string wmsTemplate(R"RAW(<GDAL_WMS>
  <Service name="TMS">
    <ServerUrl>%1%</ServerUrl>
    <Transparent>%2%</Transparent>
  </Service>
  <DataWindow>
    <UpperLeftX>%3%</UpperLeftX>
    <UpperLeftY>%4%</UpperLeftY>
    <LowerRightX>%5%</LowerRightX>
    <LowerRightY>%6%</LowerRightY>
    <TileLevel>%7%</TileLevel>
    <TileCountX>2</TileCountX>
    <TileCountY>1</TileCountY>
    <YOrigin>top</YOrigin>
  </DataWindow>
  <Projection>%8%</Projection>
  <BlockSizeX>256</BlockSizeX>
  <BlockSizeY>256</BlockSizeY>
  <BandsCount>4</BandsCount>
  <OverviewCount>%9%</OverviewCount>
  <Cache />
</GDAL_WMS>
)RAW");

void writeWms(std::ostream &os, const TmsWindyty::DatasetConfig &config
              , std::time_t time)
{
    // get current time
    struct ::tm tm;
    gmtime_r(&time, &tm);

    // format template
    char buf[2048];
    ::strftime(buf, sizeof(buf), config.urlTemplate.c_str(), &tm);
    std::string url(buf);

    os << (boost::format(wmsTemplate)
           % url
           % (config.transparent ? "TRUE" : "FALSE")
           % boost::io::group(std::fixed, config.extents.ll(0))
           % boost::io::group(std::fixed, config.extents.ur(1))
           % boost::io::group(std::fixed, config.extents.ur(0))
           % boost::io::group(std::fixed, config.extents.ll(1))
           % config.maxLod
           % config.srs
           % config.overviews);
}

TmsWindyty::File writeWms(const fs::path &root
                          , const TmsWindyty::DatasetConfig &config
                          , const Resource &resource
                          , std::time_t time
                          , int expires, int offset)
{
    auto path(root / "windyty" / resource.id.referenceFrame
              / resource.id.group /
              str(boost::format("%s.%s.xml") % resource.id.id % time));
    fs::create_directories(path.parent_path());

    // temporary file
    auto tmpPath(utility::addExtension(path, ".tmp"));
    TmsWindyty::File tmpFile(time, tmpPath.string());

    // write content to file
    {
        std::ofstream os(tmpPath.string()
                         , std::fstream::out | std::fstream::trunc);
        os.exceptions(std::ios::badbit | std::ios::failbit);
        writeWms(os, config, time + (offset * config.period));
        os.flush();
    }

    // move tmp file to proper path, release tmpFile erasure and go on
    fs::rename(tmpFile.path, path);
    tmpFile.path.clear();

    // NB: we have to store the start time of the next period
    return TmsWindyty::File(time + (expires * config.period)
                            , path.string());
}

} // namespace

void TmsWindyty::File::remove()
{
    if (!path.empty()) { fs::remove(path); }
}

TmsWindyty::TmsWindyty(const Params &params)
    : TmsRaster(params)
    , definition_(resource().definition<Definition>())
    , dsConfig_(loadConfig(absoluteDataset(definition_.dataset)))
{
    ds_.current = writeWms(config().tmpRoot, dsConfig_, resource()
                           , normalizedTime(std::time(nullptr), dsConfig_)
                           , 1, definition_.forecastOffset);
}

bool TmsWindyty::transparent_impl() const
{
    return dsConfig_.transparent;
}

bool TmsWindyty::hasMask_impl() const
{
    // we do not want metatiles and mask
    return false;
}

TmsWindyty::DsInfo TmsWindyty::dsInfo(std::time_t now) const
{
    // lock access
    std::unique_lock<std::mutex> lock(ds_.mutex);

    if (now > ds_.current.timestamp) {
        // Generate new file
        auto file(writeWms(config().tmpRoot, dsConfig_, resource()
                           , normalizedTime(now, dsConfig_)
                           , 1, definition_.forecastOffset));
        ds_.prev = std::move(ds_.current);
        ds_.current = std::move(file);
    }

    // done
    return { ds_.current.path, ds_.current.timestamp };
}

TmsRaster::DatasetDesc TmsWindyty::dataset_impl() const
{
    const auto now(std::time(nullptr));
    const auto info(dsInfo(now));

    // max age is time remaining till the end of this forecast
    long tillEnd(info.timestamp - now);

    // done
    return { info.path, tillEnd, true };
}

vr::BoundLayer TmsWindyty::boundLayer(ResourceRoot root) const
{
    // ask parent for bound layer generation
    auto bl(TmsRaster::boundLayer(root));

    // grab current dataset info
    const auto info(dsInfo(std::time(nullptr)));

    // build revision
    const auto revision(utility::format("%s", info.timestamp));

    const auto addRevision([&](std::string &url)
    {
        url.push_back("&?"[url.find('?') == std::string::npos]);
        url.append(revision);
    });

    // append revision
    addRevision(bl.url);
    if (bl.maskUrl) { addRevision(*bl.maskUrl); }
    if (bl.metaUrl) { addRevision(*bl.metaUrl); }

    bl.options = definition_.options;

    return bl;
}

} // namespace generator
