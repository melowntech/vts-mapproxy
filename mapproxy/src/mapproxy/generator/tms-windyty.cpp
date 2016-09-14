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

#include "imgproc/rastermask/cvmat.hpp"

#include "../error.hpp"

#include "./factory.hpp"
#include "./tms-windyty.hpp"

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

    virtual DefinitionBase::pointer definition() {
        return std::make_shared<TmsRaster::Definition>();
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (Resource::Generator(Resource::Generator::Type::tms, "tms-windyty")
         , std::make_shared<Factory>());
});

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

    // move to next period if inside
    if (fixed < time) { fixed += config.period; }

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
                          , std::time_t time)
{
    auto path(root / "windyty" / "XXXXXX");
    fs::create_directories(path.parent_path());
    auto buf([&]() -> std::vector<char>
    {
        auto tmp(path.string());
        return std::vector<char>(tmp.c_str(), tmp.c_str() + tmp.size() + 1);
    }());

    utility::Filedes fd(::mkstemp(buf.data()));
    if (!fd) {
        std::system_error e(errno, std::system_category());
        LOG(err3) << "Failed to create temporary file at " << path << ": <"
                  << e.code() << ", " << e.what() << ">.";
        throw e;
    }

    fs::path tmpPath(buf.data());
    fs::remove(tmpPath);

    // write data into temporary file
    {
        bio::stream_buffer<bio::file_descriptor_sink>
            buffer(fd.get(), bio::file_descriptor_flags::never_close_handle);
        std::ostream os(&buffer);
        os.exceptions(std::ios::badbit | std::ios::failbit);

        writeWms(os, config, time);
        os.flush();
    }

    // return file information
    return TmsWindyty::File(time, ::getpid(), std::move(fd));
}

} // namespace

TmsWindyty::TmsWindyty(const Params &params)
    : TmsRaster(params)
    , dsConfig_(loadConfig(absoluteDataset(definition().dataset)))
{
    ds_.current = writeWms(config().tmpRoot, dsConfig_
                           , normalizedTime(std::time(nullptr), dsConfig_));
}

bool TmsWindyty::transparent_impl() const
{
    return dsConfig_.transparent;
}

TmsRaster::DatasetDesc TmsWindyty::dataset_impl() const
{
    struct Info {
        std::string path;
        std::time_t timestamp;
    };

    auto now(std::time(nullptr));
    auto info([&]() -> Info
    {
        // lock access
        std::unique_lock<std::mutex> lock(ds_.mutex);

        if (now > ds_.current.timestamp) {
            // generate new (in use previous dataset as a place holder)
            ds_.prev = writeWms(config().tmpRoot, dsConfig_
                                , normalizedTime(now, dsConfig_));

            // everything is fine, swap them
            std::swap(ds_.current, ds_.prev);
        }

        // done
        return { ds_.current.path, ds_.current.timestamp };
    }());

    // max age is time remaining till the end of this forecast
    long tillEnd(info.timestamp - now);

    // not less than 5 minutes
    auto maxAge(std::max(tillEnd, long(300)));

    // and not more than 1 hour
    maxAge = std::min(maxAge, long(3600));

    // done
    return { info.path, maxAge };
}

} // namespace generator
