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

#include "utility/streams.hpp"
#include "utility/buildsys.hpp"
#include "utility/openmp.hpp"
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
           , const boost::optional<Color> &c)
{
    if (c) { return os << *c; }
    return os << "none";
}

class VrtWo : public service::Cmdline {
public:
    VrtWo()
        : service::Cmdline("generageVrtWo", BUILD_TARGET_VERSION)
        , minOvrSize_(256, 256), wrapx_(false), overwrite_(false)
    {
    }

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    fs::path input_;
    fs::path output_;
    math::Size2 blockSize_;
    geo::GeoDataset::Resampling resampling_;
    fs::path ovrPath_;
    math::Size2 minOvrSize_;
    bool wrapx_;
    bool overwrite_;
    boost::optional<Color> background_;
};

void VrtWo::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Path to input GDAL dataset.")
        ("output", po::value(&output_)->required()
         , "Path to output GDAL dataset.")
        ("blockSize", po::value(&blockSize_)->required()
        , "Block size.")
        ("resampling", po::value(&resampling_)->required()
        , "Resampling algorithm.")
        ("ovrPath", po::value(&ovrPath_)->required()
        , "Path where to store generated overviews.")
        ("minOvrSize", po::value(&minOvrSize_)->required()
         ->default_value(minOvrSize_)
        , "Minimum size of generated overview.")
        ("overwrite", po::value(&overwrite_)->required()
         ->default_value(false)->implicit_value(true)
        , "Overwrite existing dataset.")
        ("wrapx", po::value(&wrapx_)->required()
         ->default_value(false)->implicit_value(true)
        , "Wrap dataset in X direction.")
        ("background", po::value<Color>()
        , "Background color to use in place outside of valid "
         "data in input dataset and generated overviews.")
        ;

    pd.add("input", 1)
        .add("output", 1)
        ;

    (void) config;
}

void VrtWo::configure(const po::variables_map &vars)
{
    if (vars.count("background")) {
        background_ = vars["background"].as<Color>();
    }

    LOG(info3, log_)
        << "Config:"
        << "\n\tinput = " << input_
        << "\n\toutput = " << output_
        << "\n\tblockSize = " << blockSize_
        << "\n\tresampling = " << resampling_
        << "\n\tovrPath = " << ovrPath_
        << "\n\tminOvrSize = " << minOvrSize_
        << "\n\twrapx = " << wrapx_
        << "\n\tbackground = " << background_
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
        , maxSourceIndices_(bandCount_, 0)
    {}

    void flush() { ds_.flush(); }

    /** NB: band and srcBand are zero-based!
     */
    void addSimpleSource(int band, const fs::path &filename
                         , const geo::GeoDataset &ds
                         , int srcBand
                         , const OptionalRect &srcRect = boost::none
                         , const OptionalRect &dstRect = boost::none);

private:
    geo::GeoDataset ds_;
    int bandCount_;
    std::vector<int> maxSourceIndices_;
};

void VrtDataset::addSimpleSource(int band, const fs::path &filename
                                 , const geo::GeoDataset &ds
                                 , int srcBand
                                 , const OptionalRect &srcRect
                                 , const OptionalRect &dstRect)
{
    Rect src(srcRect ? *srcRect : Rect(ds.size()));
    Rect dst(dstRect ? *dstRect : src);

    std::ostringstream os;

    auto writeRect([&](const char *name, const Rect &r)
    {
        os <<  "<" << name << " xOff=\"" << r.origin(0)
           << "\" yOff=\"" << r.origin(1)
           << "\" xSize=\"" << r.size.width
           << "\" ySize=\"" << r.size.height << "\" />";
    });

    os << "<SimpleSource>\n"
       << "<SourceFilename relativeToVRT=\""
       << int(!filename.is_absolute())
       << "\">" << filename.string() << "</SourceFilename>\n"
       << "<SourceBand>" << (srcBand + 1) << "</SourceBand>\n"
        ;

    writeRect("SrcRect", src);
    writeRect("DstRect", dst);

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

    geo::GeoDataset::Metadata
        md(str(boost::format("source_%d") % maxSourceIndices_[band]++)
           , os.str());

    ds_.setMetadata(band + 1, md, "new_vrt_sources");
}

int VrtWo::run()
{

    auto in(geo::GeoDataset::open(input_));

    fs::create_directories(ovrPath_);

    auto srcSize(in.size());
    auto extents(in.extents());
    auto setup(makeSetup(srcSize, extents, minOvrSize_, wrapx_));

    // create virtual output dataset
    VrtDataset out(output_, in.srs(), setup.extents
                   , setup.size, in.getFormat(), in.rawNodataValue());

    const auto bands(in.bandCount());

    // add background
    if (background_) {
        Color background(*background_);
        background.resize(bands);
        const fs::path bgPath(ovrPath_ / "bg.solid");

        gdal_drivers::SolidDataset::Config cfg;
        cfg.srs = in.srs();
        cfg.size = setup.size;
        for (std::size_t i(0); i != bands; ++i) {
            const auto bp(in.bandProperties(i));

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
        for (int i(0), ei(in.bandCount()); i != ei; ++i) {
            out.addSimpleSource(i, bgPath, bg, i);
        }
    }

    auto inSize(in.size());
    for (std::size_t i(0); i != bands; ++i) {
        if (wrapx_) {
            // wrapping in x

            // add center section
            Rect centerDst(math::Point2i(setup.xPlus, 0), inSize);
            out.addSimpleSource(i, input_, in, i, boost::none
                                , centerDst);
            math::Size2 strip(math::Size2(setup.xPlus, inSize.height));

            Rect rightSrc(math::Point2i(inSize.width - setup.xPlus, 0)
                          , strip);
            Rect leftDst(math::Size2(setup.xPlus, inSize.height));
            out.addSimpleSource(i, input_, in, i, rightSrc, leftDst);

            Rect leftSrc(strip);
            Rect rightDst(math::Point2i(inSize.width + setup.xPlus, 0)
                          , strip);
            out.addSimpleSource(i, input_, in, i, leftSrc, rightDst);
        } else {
            out.addSimpleSource(i, input_, in, i);
        }
    }

    out.flush();

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    // force VRT not to share undelying datasets
    geo::Gdal::setOption("VRT_SHARED_SOURCE", 0);
    return VrtWo()(argc, argv);
}
