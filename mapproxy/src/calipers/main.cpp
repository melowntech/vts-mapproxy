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

#include "cpl_minixml.h"

#include "utility/streams.hpp"
#include "utility/buildsys.hpp"
#include "utility/openmp.hpp"
#include "utility/raise.hpp"
#include "utility/duration.hpp"
#include "utility/time.hpp"
#include "utility/enum-io.hpp"

#include "service/cmdline.hpp"

#include "geo/geodataset.hpp"
#include "geo/gdal.hpp"

#include "gdal-drivers/register.hpp"
#include "gdal-drivers/solid.hpp"

#include "vts-libs/registry.hpp"
#include "vts-libs/registry/po.hpp"
#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/csconvertor.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/math.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;
namespace ublas = boost::numeric::ublas;
namespace vr = vadstena::registry;
namespace vts = vadstena::vts;

UTILITY_GENERATE_ENUM(DatasetType,
    ((dem))
    ((ophoto))
)

class Calipers : public service::Cmdline {
public:
    Calipers()
        : service::Cmdline("calipers", BUILD_TARGET_VERSION)
        , demToOphotoScale_(3.0)
    {
    }

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    fs::path dataset_;
    std::string referenceFrameId_;
    boost::optional<DatasetType> datasetType_;
    double demToOphotoScale_;

    const vr::ReferenceFrame *referenceFrame_;
};

void Calipers::configuration(po::options_description &cmdline
                            , po::options_description &config
                            , po::positional_options_description &pd)
{
    vr::registryConfiguration(cmdline, vr::defaultPath());

    cmdline.add_options()
        ("dataset", po::value(&dataset_)->required()
         , "Path to GDAL dataset to examine.")
        ("referenceFrame", po::value(&referenceFrameId_)->required()
         , "Reference frame.")
        ("datasetType", po::value<DatasetType>()
         , "Dataset type (dem or ophoto). Mandatory only "
         "if autodetect fails.")
        ("demToOphotoScale", po::value(&demToOphotoScale_)
         ->default_value(demToOphotoScale_)->required()
         , "Inverse scale between DEM's resolution and resolution of "
         "most detailed orthophoto that can be draped on it. "
         "Used for bottom LOD calculation. "
         "To get 2x better ophoto (i.e. resolution scale 1/2) use 2.")
        ;

    pd.add("dataset", 1)
        .add("referenceFrame", 1)
        ;

    (void) config;
}

void Calipers::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    dataset_ = fs::absolute(dataset_);

    referenceFrame_ = &vr::system.referenceFrames(referenceFrameId_);

    if (vars.count("datasetType")) {
        datasetType_ = vars["datasetType"].as<DatasetType>();
    }

    LOG(info3, log_)
        << "Config:"
        << "\n\tdataset = " << dataset_
        << "\n\treferenceFrame = " << referenceFrameId_
        << "\n"
        ;

    (void) vars;
}

bool Calipers::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("calipers dataset referenceFrame [options]\n"
                "    Measures GDAL dataset in given reference frame.\n"
                "\n"
                );

        return true;
    }

    return false;
}

DatasetType detectType(const geo::GeoDataset::Descriptor &ds
                       , const boost::optional<DatasetType> &forcedType)
{
    if (forcedType) { return *forcedType; }

    if (ds.bands >= 3) { return DatasetType::ophoto; }

    if (ds.bands != 1) {
        LOGTHROW(err2, std::runtime_error)
            << "Cannot autodetect dataset type, unsupported number of bands ("
            << ds.bands << ").";
    }

    if (ds.dataType == GDT_Byte) {
        // probably monochromatic orthophoto
        return DatasetType::ophoto;
    }

    // anything else is DEM
    return DatasetType::dem;
}

struct Node {
    vts::NodeInfo node;
    cv::Mat_<char> grid;
    vts::Lod localLod;
    vts::Lod lod;

    Node(const vts::NodeInfo &node, std::size_t steps)
        : node(node), grid(steps, steps, char(0))
    {}

    void setLod(vts::Lod l) {
        localLod = l;
        lod = node.nodeId().lod + l;

        LOG(info4) << "<" << node.srs() << ">: " << lod;
    }

    typedef std::vector<Node> list;
};

int Calipers::run()
{
    const auto ds(geo::GeoDataset::open(dataset_).descriptor());

    const auto datasetType(detectType(ds, datasetType_));

    // inverse GSD scale
    const double invGsdScale((datasetType == DatasetType::dem)
                             ? demToOphotoScale_
                             : 1.0);

    // number of max steps to process
    int steps(100);

    // calculate step size
    auto es(size(ds.extents));
    const math::Size2f step((es.width / steps), (es.height / steps));

    // calculate pixel and halfpixel size
    const math::Size2f px((es.width / ds.size.height)
                          , (es.width / ds.size.height));
    const math::Size2f hpx(px.width / 2.0, px.height / 2.0);

    // center of dataset
    const auto dsCenter(math::center(ds.extents));

    const auto nodes(vts::NodeInfo::nodes(*referenceFrame_));

    UTILITY_OMP(parallel for)
    for (std::size_t nodeIndex = 0; nodeIndex < nodes.size(); ++nodeIndex) {
        const auto &node(nodes[nodeIndex]);
        const auto &nodeId(node.nodeId());

        const auto paneArea(math::area(math::size(node.extents())));
        const vts::CsConvertor conv(ds.srs, node.srs());

        /** Add projected corner. Returns true on failure.
         */
        auto convert([&](math::Point2 &c, double x, double y) -> bool
        {
            try {
                // try to convert corner
                c = conv(math::Point2d(x, y));
                // check if it is inside the node
                if (!node.inside(c)) { return true; }
            } catch (...) {
                return true;
            }
            return false;
        });

        Node validNode(node, steps);

        // best (local) LOD computed for this node
        // make_optional used to get rid of "maybe uninitialized" GCC warning
        auto bestLod(boost::make_optional<double>(false, 0.0));

        double bestDistance(std::numeric_limits<double>::max());

        math::Point2d dummy;

        // process whole grid
        double y(ds.extents.ll(1));
        for (int j(0); j < steps; ++j, y += step.height) {
            double x(ds.extents.ll(0));
            for (int i(0); i < steps; ++i, x += step.width) {
                // compute distance from center
                const math::Point2d pxCenter(x + hpx.width, y + hpx.height);

                // try to convert grid point to node's SRS
                if (convert(dummy, pxCenter(0), pxCenter(1))) {
                    continue;
                }

                // valid grid point, mark
                validNode.grid(j, i) = true;

                // convert pixel aroung grid point to node's SRS
                std::array<math::Point2d, 4> corners;
                if (convert(corners[0], x, y)
                    || convert(corners[1], x, y + px.height)
                    || convert(corners[2], x + px.width, y + px.height)
                    || convert(corners[3], x + px.width, y))
                {
                    continue;
                }

                // we have valid quadrilateral

                // calculate distance between pixel center and dataset center
                const auto distance(ublas::norm_2(pxCenter - dsCenter));

                // futher than previous best point?
                if (distance >= bestDistance) { continue; }

                // calculate (approximate) projected triangle area
                const auto pxArea
                    (vts::triangleArea(corners[0], corners[1], corners[2])
                     + vts::triangleArea(corners[2], corners[3], corners[0]));

                // calculate best lod:
                // divide node's pane area by tiles area
                // apply square root to get number of tiles per side
                // and log2 to get lod
                // NB: log2(sqrt(a)) = 0.5 * log2(a)
                // NB: inverse GSD scale is applied to node pane area
                const auto lod(0.5 * std::log2
                               ((paneArea * invGsdScale * invGsdScale)
                                / (pxArea * vr::BoundLayer::tileArea())));
                // sanity check: no negative LOD
                if (lod >= 0.0) {
                    bestLod = lod;
                    bestDistance = distance;
                }
            }
        }

        if (!bestLod) { continue; }

        // round to closest integral LOD
        vts::Lod lod(std::round(*bestLod));

        // check whether curren subtree root can produce tiles at computed
        // lod
        if (!compatible
            (vts::NodeInfo(node.referenceFrame()
                           , vts::lowestChild(nodeId, lod))
             , node)) { continue; }


        // finally!
        validNode.setLod(lod);
    }

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    gdal_drivers::registerAll();
    return Calipers()(argc, argv);
}
