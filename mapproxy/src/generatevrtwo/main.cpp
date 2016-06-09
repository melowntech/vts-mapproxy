#include <cstdlib>
#include <utility>
#include <functional>
#include <map>

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include "gdal/cpl_minixml.h"

#include "utility/streams.hpp"
#include "utility/buildsys.hpp"
#include "utility/openmp.hpp"
#include "utility/raise.hpp"
#include "service/cmdline.hpp"

#include "geo/geodataset.hpp"
#include "geo/gdal.hpp"

#include "gdal-drivers/register.hpp"
#include "gdal-drivers/solid.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

struct Color {
    std::vector<double> value;

    double operator[](std::size_t i) const { return value[i]; }

    /** Resizes Color to given number of channels.
     *  If current size is larger than requested: crops
     *  If current size is lower than requested: replicates last one
     *  If current size is empty: generates color full of zeroes
     */
    void resize(std::size_t size) {
        if (value.empty() || (size <= value.size())) {
            value.resize(size);
            return;
        }

        auto last(value.back());
        value.resize(size, last);
    }

    typedef boost::optional<Color> optional;
};

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is, Color &c)
{
    double v;
    auto comma(utility::match<CharT>(','));

    while (is) {
        if (!(is >> v)) { break; }
        c.value.push_back(v);
        if (!(is >> comma)) { break; }
        if (!comma.matched) { break; }
    }

    return is;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Color &c)
{
    bool first(true);
    for (auto value : c.value) {
        if (!first) {
            os << ',';
        } else {
            first = false;
        }
        os << value;
    }
    return os;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const Color::optional &c)
{
    if (c) { return os << *c; }
    return os << "none";
}

struct Config {
    std::string input;
    fs::path output;
    math::Size2 tileSize;
    geo::GeoDataset::Resampling resampling;
    fs::path ovrPath;
    math::Size2 minOvrSize;
    bool wrapx;
    bool overwrite;
    Color::optional background;

    Config()
        : minOvrSize(256, 256), wrapx(false), overwrite(false)
    {}
};

class VrtWo : public service::Cmdline {
public:
    VrtWo()
        : service::Cmdline("generageVrtWo", BUILD_TARGET_VERSION)
    {
    }

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    Config config_;
};

void VrtWo::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&config_.input)->required()
         , "Path to input GDAL dataset.")
        ("output", po::value(&config_.output)->required()
         , "Path to output GDAL dataset.")
        ("tileSize", po::value(&config_.tileSize)->required()
        , "Tile size.")
        ("resampling", po::value(&config_.resampling)->required()
        , "Resampling algorithm.")
        ("ovrPath", po::value(&config_.ovrPath)->required()
        , "Path where to store generated overviews.")
        ("minOvrSize", po::value(&config_.minOvrSize)->required()
         ->default_value(config_.minOvrSize)
        , "Minimum size of generated overview.")
        ("overwrite", po::value(&config_.overwrite)->required()
         ->default_value(false)->implicit_value(true)
        , "Overwrite existing dataset.")
        ("wrapx", po::value(&config_.wrapx)->required()
         ->default_value(false)->implicit_value(true)
        , "Wrap dataset in X direction.")
        ("background", po::value<Color>()
        , "Optional background. If whole warped tile contains this "
         "color it is left empty in the output. Solid dataset with this color "
         "is created and places as a first source for each band in "
         "all overvies.")
        ;

    pd.add("input", 1)
        .add("output", 1)
        ;

    (void) config;
}

void VrtWo::configure(const po::variables_map &vars)
{
    if (vars.count("background")) {
        config_.background = vars["background"].as<Color>();
    }

    LOG(info3, log_)
        << "Config:"
        << "\n\tinput = " << config_.input
        << "\n\toutput = " << config_.output
        << "\n\ttileSize = " << config_.tileSize
        << "\n\tresampling = " << config_.resampling
        << "\n\tovrPath = " << config_.ovrPath
        << "\n\tminOvrSize = " << config_.minOvrSize
        << "\n\twrapx = " << config_.wrapx
        << "\n\tbackground = " << config_.background
        << "\n"
        ;
}

bool VrtWo::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("generatevrtwo input output [options]\n"
                "    Generates virtual GDAL dataset with overviews.\n"
                "\n"
                );

        return true;
    }

    return false;
}

typedef std::vector<math::Size2> Sizes;

struct Setup {
    math::Size2 size;
    math::Extents2 extents;
    Sizes ovrSizes;
    int xPlus;

    Setup() : xPlus() {}
};

Setup makeSetup(math::Size2 size, math::Extents2 extents
                , const math::Size2 &minSize
                , bool wrapx)
{
    auto halve([&]()
    {
        size.width = int(std::round(size.width / 2.0));
        size.height = int(std::round(size.height / 2.0));
    });

    Setup setup;
    setup.extents = extents;
    setup.size = size;

    halve();
    while ((size.width >= minSize.width)
           || (size.height >= minSize.height))
    {
        setup.ovrSizes.push_back(size);
        halve();
    }

    if (!wrapx) { return setup; }

    // add one pixel to each side at bottom level and double on the way up
    int add(2);
    for (auto &s : boost::adaptors::reverse(setup.ovrSizes)) {
        s.width += add;
        add *= 2;
    }

    // set x plus component
    setup.xPlus = add / 2;

    // calculate pixel width
    auto es(math::size(setup.extents));
    auto pw(es.width / setup.size.width);

        // calculate addition
    auto eadd(setup.xPlus * pw);

    // apply addition in both dimensions
    setup.extents.ll(0) -= eadd;
    setup.extents.ur(0) += eadd;

    // and finally update size
    setup.size.width += add;

    return setup;
}

geo::GeoDataset::Format asVrt(geo::GeoDataset::Format &f)
{
    f.storageType = geo::GeoDataset::Format::Storage::vrt;
    return f;
}

struct Rect {
    math::Point2i origin;
    math::Size2 size;

    Rect(const math::Point2i &origin = math::Point2i()
         , const math::Size2 &size = math::Size2())
        : origin(origin), size(size)
    {}

    Rect(const math::Size2 &size) : origin(), size(size) {}
};

typedef boost::optional<Rect> OptionalRect;

class VrtDataset {
public:
    VrtDataset(const fs::path &path, const geo::SrsDefinition &srs
               , const math::Extents2 &extents, const math::Size2 &size
               , geo::GeoDataset::Format format
               , const geo::GeoDataset::NodataValue &nodata)
        : ds_(geo::GeoDataset::create
              (path, srs, extents, size, asVrt(format), nodata))
        , bandCount_(format.channels.size())
    {}

    void flush() { ds_.flush(); }

    /** NB: band and srcBand are zero-based!
     */
    void addSimpleSource(int band, const fs::path &filename
                         , const geo::GeoDataset &ds
                         , int srcBand
                         , const OptionalRect &srcRect = boost::none
                         , const OptionalRect &dstRect = boost::none);

    void addOverview(int band, const fs::path &filename, int srcBand);

    void addBackground(const fs::path &path, const Color::optional &color
                       , const boost::optional<fs::path> &localTo
                       = boost::none);

    const geo::GeoDataset& dataset() const { return ds_; }

    std::size_t bandCount() const { return bandCount_; };

private:
    geo::GeoDataset ds_;
    std::size_t bandCount_;
};

void writeSourceFilename(std::ostream &os, const fs::path &filename
                         , bool shared)
{
    os << "<SourceFilename relativeToVRT=\""
       << int(!filename.is_absolute())
       << " shared=" << int(shared)
       << "\">" << filename.string() << "</SourceFilename>\n"
        ;
}

void writeSourceBand(std::ostream &os, int srcBand)
{
    os << "<SourceBand>" << (srcBand + 1) << "</SourceBand>\n";
}

void writeRect(std::ostream &os, const char *name, const Rect &r)
{
    os <<  "<" << name << " xOff=\"" << r.origin(0)
       << "\" yOff=\"" << r.origin(1)
       << "\" xSize=\"" << r.size.width
       << "\" ySize=\"" << r.size.height << "\" />";
}

void VrtDataset::addSimpleSource(int band, const fs::path &filename
                                 , const geo::GeoDataset &ds
                                 , int srcBand
                                 , const OptionalRect &srcRect
                                 , const OptionalRect &dstRect)
{
    Rect src(srcRect ? *srcRect : Rect(ds.size()));
    Rect dst(dstRect ? *dstRect : src);

    std::ostringstream os;

    os << "<SimpleSource>\n";

    writeSourceFilename(os, filename, true);
    writeSourceBand(os, srcBand);

    writeRect(os, "SrcRect", src);
    writeRect(os, "DstRect", dst);

    const auto bp(ds.bandProperties(srcBand));

    os << "<SourceProperties RasterXSize=\""<< bp.size.width
       << "\" RasterYSize=\"" << bp.size.height
       << "\" DataType=\"" << bp.dataType
       << "\" BlockXSize=\"" << bp.blockSize.width
       << "\" BlockYSize=\"" << bp.blockSize.height
       << "\" />\n"
        ;

    os << "</SimpleSource>\n"
        ;

    ds_.setMetadata(band + 1, geo::GeoDataset::Metadata("source", os.str())
                    , "new_vrt_sources");
}

void VrtDataset::addOverview(int band, const fs::path &filename, int srcBand)
{
    std::ostringstream os;
    os << "<Overview>\n";
    writeSourceFilename(os, filename, true);
    writeSourceBand(os, srcBand);
    os << "</Overview>\n";
    ds_.setMetadata(band + 1, geo::GeoDataset::Metadata("overview", os.str())
                    , "new_vrt_sources");
}

void VrtDataset::addBackground(const fs::path &path
                               , const Color::optional &color
                               , const boost::optional<fs::path> &localTo)
{
    if (!color) { return; }

    auto background(*color);
    background.resize(bandCount_);
    const fs::path fname("bg.solid");
    const fs::path bgPath(path / fname);
    const fs::path storePath(localTo ? (*localTo / fname) : bgPath);

    gdal_drivers::SolidDataset::Config cfg;
    cfg.srs = ds_.srs();
    cfg.size = ds_.size();
    cfg.geoTransform(ds_.geoTransform());
    for (std::size_t i(0); i != bandCount_; ++i) {
        const auto bp(ds_.bandProperties(i));

        gdal_drivers::SolidDataset::Config::Band band;
        band.value = background[i];
        band.colorInterpretation = bp.colorInterpretation;
        band.dataType = bp.dataType;
        cfg.bands.push_back(band);
    }

    // create background dataset
    auto bg(geo::GeoDataset::use
            (gdal_drivers::SolidDataset::create(bgPath, cfg)));

    // map layers
    for (std::size_t i(0); i != bandCount_; ++i) {
        addSimpleSource(i, storePath, bg, i);
    }
}

void addOverview(const fs::path &vrtPath, const fs::path &ovrPath)
{
    typedef std::shared_ptr< ::CPLXMLNode> XmlNode;
    auto xmlNode([](::CPLXMLNode *n) -> XmlNode
    {
        if (!n) {
            LOGTHROW(err1, std::runtime_error)
                << "Cannot parse XML: <" << ::CPLGetLastErrorMsg() << ">.";
        }
        return XmlNode(n, [](::CPLXMLNode *n) { ::CPLDestroyXMLNode(n); });
    });

    class NodeIterator {
    public:
        NodeIterator(::CPLXMLNode *node, const char *name = nullptr)
            : node_(node->psChild), name_(name)
        {
            // go till node with given name is hit
            while (node_ && !matches()) {
                node_ = node_->psNext;
            }
        }

        operator bool() const { return node_; }
        ::CPLXMLNode* operator*() { return node_; }
        ::CPLXMLNode* operator->() { return node_; }

        NodeIterator& operator++() {
            if (!node_) { return *this; }
            // skip current node and find new with the same name
            do {
                node_ = node_->psNext;
            } while (node_ && !matches());
            return *this;
        }

    private:
        bool matches() const {
            return !name_ || !std::strcmp(name_, node_->pszValue);
        }

        ::CPLXMLNode *node_;
        const char *name_;
    };

    auto root(xmlNode(::CPLParseXMLFile(vrtPath.c_str())));

    for (NodeIterator ni(root.get(), "VRTRasterBand"); ni; ++ni) {
        NodeIterator bandNode(*ni, "band");
        if (!bandNode) {
            LOG(warn3) << "Cannot find band attribute in VRTRasterBand.";
            continue;
        }

        // get band number
        auto band(bandNode->psChild->pszValue);

        auto overview(::CPLCreateXMLNode(*ni, ::CXT_Element, "Overview"));
        auto sourceFilename(::CPLCreateXMLNode
                            (overview, ::CXT_Element, "SourceFilename"));
        auto relativeToVRT(::CPLCreateXMLNode
                           (sourceFilename, CXT_Attribute, "relativeToVRT"));
        ::CPLCreateXMLNode(relativeToVRT, CXT_Text
                           , (ovrPath.is_absolute() ? "0" : "1"));
        ::CPLCreateXMLNode(sourceFilename, ::CXT_Text, ovrPath.c_str());
        auto sourceBand(::CPLCreateXMLNode
                        (overview, ::CXT_Element, "SourceBand"));
        ::CPLCreateXMLNode(sourceBand, ::CXT_Text, band);
    }

    auto res(::CPLSerializeXMLTreeToFile(root.get(), vrtPath.c_str()));
    if (!res) {
        LOGTHROW(err3, std::runtime_error)
            << "Cannot save updated VRT file into " << vrtPath << ".";
    }
}

Setup buildDatasetBase(const Config &config)
{
    LOG(info3) << "Creating dataset base in " << config.output << ".";
    auto in(geo::GeoDataset::open(config.input));

    auto setup(makeSetup(in.size(), in.extents(), config.minOvrSize
                         , config.wrapx));

    // create virtual output dataset
    VrtDataset out(config.output, in.srs(), setup.extents
                   , setup.size, in.getFormat(), in.rawNodataValue());

    // add input bands
    auto inSize(in.size());
    for (std::size_t i(0); i != in.bandCount(); ++i) {
        if (config.wrapx) {
            // wrapping in x

            // add center section
            Rect centerDst(math::Point2i(setup.xPlus, 0), inSize);
            out.addSimpleSource(i, config.input, in, i, boost::none
                                , centerDst);
            math::Size2 strip(math::Size2(setup.xPlus, inSize.height));

            Rect rightSrc(math::Point2i(inSize.width - setup.xPlus, 0)
                          , strip);
            Rect leftDst(math::Size2(setup.xPlus, inSize.height));
            out.addSimpleSource(i, config.input, in, i, rightSrc, leftDst);

            Rect leftSrc(strip);
            Rect rightDst(math::Point2i(inSize.width + setup.xPlus, 0)
                          , strip);
            out.addSimpleSource(i, config.input, in, i, leftSrc, rightDst);
        } else {
            out.addSimpleSource(i, config.input, in, i);
        }
    }

    out.flush();

    return setup;
}

struct TIDGuard {
    TIDGuard(const std::string &id)
        : old(dbglog::thread_id())
    {
        dbglog::thread_id(id);
    }
    ~TIDGuard() { dbglog::thread_id(old); }

    const std::string old;
};

class Dataset {
public:
    Dataset(const std::string &path)
        : path_(path), ds_(geo::GeoDataset::placeholder())
    {}

    Dataset(const Dataset &d)
        : path_(d.path_), ds_(geo::GeoDataset::open(path_))
    {}

    ~Dataset() {}

    geo::GeoDataset& ds() { return ds_; }

private:
    std::string path_;
    geo::GeoDataset ds_;
};

template <typename T>
bool compareValue(const cv::Mat_<T> &block
                  , const math::Size2 &size
                  , T value)
{
    for (int j(0); j != size.height; ++j) {
        for (int i(0); i != size.width; ++i) {
            if (block(j, i) != value) { return false; }
        }
    }
    return true;
}

bool compare(const geo::GeoDataset::Block &block, const math::Size2 &size
             , ::GDALDataType type, double value)
{
    switch (type) {
    case ::GDT_Byte:
        return compareValue<std::uint8_t>(block.data, size, value);

    case ::GDT_UInt16:
        return compareValue<std::uint16_t>(block.data, size, value);

    case ::GDT_Int16:
        return compareValue<std::int16_t>(block.data, size, value);

    case ::GDT_UInt32:
        return compareValue<std::uint32_t>(block.data, size, value);

    case ::GDT_Int32:
        return compareValue<std::int32_t>(block.data, size, value);

    case ::GDT_Float32:
        return compareValue<float>(block.data, size, value);

    case ::GDT_Float64:
        return compareValue<double>(block.data, size, value);

    default:
        utility::raise<std::runtime_error>
            ("Unsupported data type <%s>.", type);
    };
    throw;
}

bool emptyTile(const Config &config, const geo::GeoDataset &ds)
{
    if (config.background) {
        // TODO: we are using a background color: need to check content for
        // exact color

        // get background
        int bands(ds.bandCount());
        auto background(*config.background);
        background.resize(bands);

        auto bps(ds.bandProperties());

        // process all blocks
        for (const auto &bi : ds.getBlocking()) {
            for (int i(0); i != bands; ++i) {
                // load block in native format
                auto block(ds.readBlock(bi.offset, i, true));
                if (!compare(block, bi.size, bps[i].dataType, background[i])) {
                    // not single color
                    return false;
                }
            }
        }

        return true;
    }

    // no background -> do not store if mask is empty

    // fetch optimized mask
    auto mask(ds.fetchMask(true));
    // no data -> full area is valid
    if (!mask.data) { return false; }

    // no non-zero count -> empty mask
    return !cv::countNonZero(mask);
}

fs::path createOverview(const Config &config, int ovrIndex
                        , const fs::path &dir, const math::Size2 &size
                        , geo::GeoDataset::Overview srcOverview)
{
    auto ovrName(dir / "ovr.vrt");

    LOG(info3) << "Creating overview in " << ovrName << ".";

    VrtDataset ovr([&]() -> VrtDataset
    {
        auto src(geo::GeoDataset::open(config.output));
        return VrtDataset(ovrName, src.srs(), src.extents()
                          , size, src.getFormat(), src.rawNodataValue());
    }());

    auto extents(ovr.dataset().extents());
    ovr.addBackground(dir, config.background, fs::path());

    const auto &ts(config.tileSize);
    math::Size2 tiled((size.width + ts.width - 1) / ts.width
                       , (size.height + ts.height - 1) / ts.height);

    // compute tile size in real extents
    auto tileSize([&]() -> math::Size2f
    {
        auto es(math::size(extents));
        return math::Size2f((es.width * ts.width) / size.width
                            , (es.height * ts.height) / size.height);
    }());
    // extent's upper-left corner is origin for tile calculations
    math::Point2 origin(ul(extents));

    auto tc(math::area(tiled));

    // last tile size
    math::Size2 lts(size.width - (tiled.width - 1) * ts.width
                    , size.height - (tiled.height - 1) * ts.height);

    Dataset dataset(config.input);

    geo::GeoDataset::WarpOptions warpOptions;
    warpOptions.overview = srcOverview;
    warpOptions.safeChunks = false;

    UTILITY_OMP(parallel for firstprivate(dataset))
        for (int i = 0; i < tc; ++i) {
            math::Point2i tile(i % tiled.width, i / tiled.width);

            bool lastX(tile(0) == (tiled.width - 1));
            bool lastY(tile(1) == (tiled.height - 1));

            math::Size2 pxSize(lastX ? lts.width : ts.width
                               , lastY ? lts.height : ts.height);

            // calculate extents
            math::Point2 ul(origin(0) + tileSize.width * tile(0)
                            , origin(1) - tileSize.height * tile(1));
            math::Point2 lr(lastX ? extents.ur(0) : ul(0) + tileSize.width
                            , lastY ? extents.ll(1): ul(1) - tileSize.height);

            math::Extents2 te(ul(0), lr(1), lr(0), ul(1));
            TIDGuard tg(str(boost::format("tile:%d-%d-%d")
                            % ovrIndex % tile(0) % tile(1)));

            LOG(info2)
                << std::fixed
                << "Processing tile " << ovrIndex
                << '-' << tile(0) << '-' << tile(1) << " (extents: " << te
                << ").";

            // try warp
            auto &src(dataset.ds());
            auto tmp(geo::GeoDataset::deriveInMemory
                     (src, src.srs(), pxSize, te));
            src.warpInto(tmp, config.resampling, warpOptions);

            // TODO: check result and skip if no need to store
            if (emptyTile(config, tmp)) {
                LOG(info3)
                    << std::fixed
                    << "Processed tile " << ovrIndex
                    << '-' << tile(0) << '-' << tile(1) << " (extents: " << te
                    << ") [empty].";
                continue;
            }

            // strore result to file
            fs::path tileName(str(boost::format("%d-%d.jp2")
                                  % tile(0) % tile(1)));
            fs::path tilePath(dir / tileName);

            // copy as a jpeg 2000 file that doesn't corrupt data
            auto dst(geo::GeoDataset::createCopy
                     (tilePath, "JP2OpenJPEG", tmp
                      , geo::GeoDataset::Options
                      ("REVERSIBLE", true)("QUALITY", 100)
                      ("BLOCKXSIZE", 256)("BLOCKYSIZE", 256)));
            dst.flush();

            // store result
            Rect drect(math::Point2i(tile(0) * ts.width, tile(1) * ts.height)
                       , pxSize);

            for (std::size_t b(0), eb(ovr.bandCount()); b != eb; ++b) {
                UTILITY_OMP(critical)
                    ovr.addSimpleSource(b, tileName, dst, b
                                        , boost::none, drect);
            }

            LOG(info3)
                << std::fixed
                << "Processed tile " << ovrIndex
                << '-' << tile(0) << '-' << tile(1) << " (extents: " << te
                << ") [valid].";
        }

    ovr.flush();

    return ovrName;
}

int VrtWo::run()
{
    fs::create_directories(config_.ovrPath);

    auto setup(buildDatasetBase(config_));

    // generate overviews
    geo::GeoDataset::Overview srcOverview;
    for (std::size_t i(0); i != setup.ovrSizes.size(); ++i) {
        auto dir(config_.ovrPath / str(boost::format("%d") % i));
        fs::create_directories(dir);

        auto path(createOverview(config_, i, dir, setup.ovrSizes[i]
                                 , srcOverview));

        // use previous level
        srcOverview = i;

        // add overview (manually by manipulating the XML)
        addOverview(config_.output, path);
    }

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    // force VRT not to share undelying datasets
    geo::Gdal::setOption("VRT_SHARED_SOURCE", 0);
    return VrtWo()(argc, argv);
}
