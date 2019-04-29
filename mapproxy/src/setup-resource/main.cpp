/**
 * Copyright (c) 2018 Melown Technologies SE
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

#include <cstdlib>
#include <utility>
#include <functional>
#include <map>
#include <boost/regex.hpp>

#include <boost/optional.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/trim.hpp>

#include "utility/streams.hpp"
#include "utility/buildsys.hpp"
#include "utility/path.hpp"
#include "utility/enum-io.hpp"
#include "utility/path.hpp"
#include "utility/filesystem.hpp"
#include "utility/format.hpp"
#include "utility/md5.hpp"

#include "service/cmdline.hpp"
#include "service/ctrlclient.hpp"
#include "service/pidfile.hpp"

#include "geo/geodataset.hpp"
#include "geo/gdal.hpp"
#include "gdal-drivers/register.hpp"

#include "vts-libs/registry/po.hpp"
#include "vts-libs/vts/tileop.hpp"
#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/tileindex.hpp"

// mapproxy stuff
#include "calipers/calipers.hpp"
#include "generatevrtwo/generatevrtwo.hpp"
#include "tiling/tiling.hpp"
#include "mapproxy/resource.hpp"
#include "mapproxy/definition.hpp"
#include "mapproxy/mapproxy.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace vts = vtslibs::vts;
namespace vr = vtslibs::registry;

UTILITY_GENERATE_ENUM_CI(ResourceType,
                         ((tms)("TMS"))
                         ((tin)("TIN"))
                         )

struct Config {
    boost::optional<std::string> geoidGrid;
    RasterFormat format;
    boost::optional<geo::GeoDataset::Resampling> tmsResampling;
    bool transparent;
    std::vector<std::string> attributions;

    int autoCreditId;

    fs::path mapproxyDataRoot;
    fs::path mapproxyDefinitionDir;
    fs::path mapproxyCtrl;

    Config()
        : format(RasterFormat::jpg)
        , transparent(false)
        , autoCreditId(32768)
    {}
};

class SetupResource : public service::Cmdline {
public:
    SetupResource()
        : service::Cmdline("mapproxy-setup-resource", BUILD_TARGET_VERSION)
    {}

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    std::string dataset_; // do not use fs::path since it needs quotes in case
                          // of spaces in filename

    boost::optional<ResourceType> resourceType_;

    Resource::Id resourceId_;

    Config config_;
};

void SetupResource::configuration(po::options_description &cmdline
                                , po::options_description &config
                                , po::positional_options_description &pd)
{
    vr::registryConfiguration(config, vr::defaultPath());

    cmdline.add_options()
        ("referenceFrame", po::value(&resourceId_.referenceFrame)->required()
         , "Reference frame.")
        ("id", po::value(&resourceId_.id)
         , "Resource ID. Deduced from filename if not provided.")
        ("group", po::value(&resourceId_.group)
         , "Resource group ID. Deduced from filename if not provided.")
        ("dataset", po::value(&dataset_)->required()
         , "Path to input raster dataset.")
        ("resourceType", po::value<ResourceType>()
         , "Resource type: TMS or TIN.")

        ("tin.geoidGrid", po::value<std::string>()
         , "TIN: Geoid grid to inject into dataset's SRS. Defaults to "
         "reference frame's body default geoid grid if not specified.")

        ("tms.format", po::value(&config_.format)
         ->default_value(config_.format)->required()
         , "TMS: image format to use when generating tiles (jpg or png).")
        ("tms.resampling", po::value<geo::GeoDataset::Resampling>()
         , utility::concat
         ("TMS: GDAL resampling used to generate tiles and overview. One of ["
          , enumerationString(geo::GeoDataset::Resampling())
          , "].").c_str())
        ("tms.transparent", po::value(&config_.transparent)
         ->default_value(config_.transparent)->required()
         , "TMS: mark tiles as trasnparent.")

        ("credits.firstNumericId", po::value(&config_.autoCreditId)
         ->default_value(config_.autoCreditId)->required()
         , "First numeric ID of the auto-generated credits.")

        ("attribution", po::value(&config_.attributions)->required()
         , "Atribution text, one per attribution. "
         "At least one attribution is required.")
        ;

    config.add_options()
        ("mapproxy.dataRoot", po::value(&config_.mapproxyDataRoot)->required()
         , "Path to mapproxy resource data root directory.")
        ("mapproxy.definitionDir"
         , po::value(&config_.mapproxyDefinitionDir)->required()
         , "Path to mapproxy resource definition directory.")
        ("mapproxy.ctrl", po::value(&config_.mapproxyCtrl)->required()
         , "Path to mapproxy control socket.")
        ;

    pd.add("dataset", 1)
        ;

    (void) pd;
}

void SetupResource::configure(const po::variables_map &vars)
{
    vr::registryConfigure(vars);

    if (vars.count("resourceType")) {
        resourceType_ = vars["resourceType"].as<ResourceType>();
    }

    if (vars.count("tin.geoidGrid")) {
        config_.geoidGrid = vars["tin.geoidGrid"].as<std::string>();
    }

    if (vars.count("tms.resampling")) {
        config_.tmsResampling = vars["tms.resampling"]
            .as<geo::GeoDataset::Resampling>();
    }

    LOG(info3, log_)
        << "Config:"
        << "\nmapproxy.dataRoot = " << config_.mapproxyDataRoot
        << "\nmapproxy.definitionDir = " << config_.mapproxyDefinitionDir
        << "\nmapproxy.ctrl = " << config_.mapproxyCtrl
        << "\nreferenceFrame = " << resourceId_.referenceFrame
        << "\ncredits.firstNumericId = " << config_.autoCreditId
        << "\n"
        ;
}

bool SetupResource::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("mapproxy resource setup tool\n"
                "    Sets up and adds new mapproxy resource to "
                "extisting mapproxy installation.\n"
                "\n"
                );

        return true;
    }

    return false;
}

struct LogLinePrefix {
    LogLinePrefix(const std::string &prefix)
        : old(dbglog::log_line_prefix())
    {
        dbglog::log_line_prefix(prefix);
    }

    ~LogLinePrefix() { dbglog::log_line_prefix(old); }

    std::string old;
};

boost::optional<calipers::DatasetType>
asDatasetType(const boost::optional<ResourceType> &type)
{
    if (!type) { return boost::none; }
    switch (*type) {
    case ResourceType::tms: return calipers::DatasetType::ophoto;
    case ResourceType::tin: return calipers::DatasetType::dem;
    }
    return boost::none;
}

Resource::Id deduceResourceId(fs::path path, Resource::Id resourceId)
{
    // pass if already valid
    if (!(resourceId.id.empty() || resourceId.group.empty())) {
        return resourceId;
    }

    path = fs::absolute(path);
    const auto dir(utility::sanitizePath(path.parent_path().filename())
                   .string());
    const auto stem(utility::sanitizePath(path.stem()).string());

    if (resourceId.group.empty()) { resourceId.group = dir; }
    if (resourceId.id.empty()) { resourceId.id = stem; }

    return resourceId;
}

std::string dropExpansion(const std::string &str)
{
    std::string out;
    bool firstNonSpace(false);
    bool dumpAnyway(false);

    char match(0);
    for (auto c : str) {
        switch (c) {
        case '{':
            if (!match) {
                match = '}';
                continue;
            }
            break;

        case '[':
            if (!match) {
                match = ']';
                continue;
            }
            break;

        default:
            if (!firstNonSpace && !std::isspace(c)) {
                firstNonSpace = true;
            }
        }

        // close if matching paren is hit
        if (c == match) {
            match = 0;
            firstNonSpace = false;
            dumpAnyway = false;
            continue;
        }

        if ((match == ']') && std::isspace(c) && firstNonSpace) {
            // second token in []
            dumpAnyway = true;
        }

        if (!match || dumpAnyway) { out.push_back(c); }
    }

    return out;
}

std::string contractAcronyms(const std::string &str)
{
    typedef boost::regex br;

    boost::regex e("(^|[^[:alnum:]])(([a-z] ?[ .])+)", br::perl | br::icase);

    const auto formatter([](const boost::smatch &what) -> std::string
    {
        std::string out(what.str(1));
        for (auto c : what.str(2)) {
            switch (c) {
            case '.': case ' ': break;
            default: out.push_back(c); break;
            }
        }
        out.push_back(' ');
        return out;
    });

    return boost::regex_replace(str, e, formatter);
}

std::string attribution2creditId(const std::string &attribution)
{
    const auto firstPass([](const std::string &str) -> std::string
    {
        utility::SanitizerOptions so(true);
        so.dashNonAlphanum = false;
        so.singleSpace = true;
        return utility::sanitizeId(str, so);
    });

    const auto secondPass([](const std::string &str) -> std::string
    {
        utility::SanitizerOptions so;
        so.latinize = false;
        so.removeAccents = false;
        return utility::sanitizeId(str, so);
    });

    return secondPass
        ("auto-" + contractAcronyms
         (firstPass(dropExpansion(attribution))));
}

vr::Credit buildCredit(const std::string &creditId
                       , const std::string &attribution
                       , vr::Credit::NumericId numericId)
{
    vr::Credit credit;
    credit.id = creditId;
    credit.numericId = numericId;

    credit.copyrighted = (attribution.find("{copy}") != std::string::npos);
    credit.notice = attribution;

    return credit;
}

typedef std::map<std::string, vr::Credit::dict> CreditFileMapping;
typedef std::set<std::string> ChangedFiles;

vr::Credit::NumericId allocateNumericId(const Config &config
                                        , const CreditFileMapping &cfm)
{
    vr::Credit::NumericId maxId(0);
    for (const auto &credits : cfm) {
        for (const auto &credit : credits.second) {
            maxId = std::max(maxId, credit.numericId);
        }
    }

    if (maxId) { return maxId + 1; }
    return config.autoCreditId;
}

vr::Credit allocateCredit(const Config &config
                          , const CreditFileMapping &cfm
                          , vr::Credit::dict &credits
                          , const std::string &creditId
                          , const std::string &attribution)
{
    // generate new credit ID (either creditId or another in case of collision)
    auto newCreditId([&]() -> std::string
    {
        auto newCreditId(creditId);

        for (int i(1); i <= 100; ++i) {
            if (cfm.find(newCreditId) == cfm.end()) { return newCreditId; }
            newCreditId = utility::format("%s.%d", creditId, i);
        }

        LOGTHROW(err3, std::runtime_error)
            << "Too much diferent versions of attribution with ID <"
            << creditId << ">.";
        throw;
    }());

    auto credit(buildCredit(newCreditId, attribution
                            , allocateNumericId(config, cfm)));
    credits.add(credit);
    return credit;
}

vr::Credit attribution2credit(const Config &config, CreditFileMapping &cfm
                              , ChangedFiles &cf
                              , const std::string &attribution)
{
    const auto creditId(attribution2creditId(attribution));
    LOG(info3) << "Using credit ID <" << creditId << "> for attribution \""
               << attribution << "\".";

    auto fcfm(cfm.find(creditId));
    if (fcfm != cfm.end()) {
        // found a file with given credit id
        auto &credits(fcfm->second);

        // try to find one with the same attributiin
        for (const auto &credit : credits) {
            if (credit.notice == attribution) { return credit; }
        }

        // none found, allocate new credit in this file
        auto credit
            (allocateCredit(config, cfm, credits, creditId, attribution));
        cf.insert(creditId);
        return credit;
    }

    // allocate new credit in completely new file
    vr::Credit::dict credits;
    auto credit(allocateCredit(config, cfm, credits, creditId, attribution));
    cfm.insert(CreditFileMapping::value_type(creditId, credits));
    cf.insert(creditId);
    return credit;
}

vr::Credit::dict attributions2credits(const Config &config)
{
    // auto generated credits home
    const auto creditHome(config.mapproxyDataRoot / "auto-credits");
    fs::create_directories(creditHome.parent_path());

    const auto pidFilePath(creditHome / "lock.pid");

    // lock credits databse using PID file
    service::pidfile::ScopedPidFile(pidFilePath, 60);

    // load credits
    CreditFileMapping cfm;
    for (fs::directory_iterator icreditHome(creditHome), ecreditHome;
         icreditHome != ecreditHome; ++icreditHome)
    {
        if (!fs::is_regular_file(icreditHome->status())) { continue; }
        const auto &path(icreditHome->path());
        if (path.extension() != ".json") { continue; }

        cfm.insert(CreditFileMapping::value_type
                   (path.stem().string(), vr::loadCredits(path)));
    }

    // process
    ChangedFiles cf;
    vr::Credit::dict credits;
    for (const auto &attribution : config.attributions) {
        credits.add(attribution2credit(config, cfm, cf, attribution));
    }

    // store changed credits
    for (const auto &changedFile : cf) {
        auto icfm(cfm.find(changedFile));
        if (icfm == cfm.end()) {
            LOG(warn4)
                << "Unexpected filename in set of changed credit "
                "config files: <" << changedFile << ">.";
            continue;
        }

        // store credits
        vr::saveCredits(creditHome / (changedFile + ".json")
                        , icfm->second);
    }

    return credits;
}


void createVrtWO(const fs::path &srcPath
                 , const fs::path &root, const fs::path &name
                 , geo::GeoDataset::Resampling resampling
                 , const calipers::Measurement &cm)
{
    vrtwo::Config config;
    config.resampling = resampling;
    config.overwrite = true;
    config.wrapx = cm.xOverlap;

    config.createOptions
        ("TILED", true)
        ("COMPRESS", "DEFLATE")
        ("PREDICTOR", "")
        ("ZLEVEL", 9)
        ;

    const auto ovrPathLocal("vttwo" / name);

    config.pathToOriginalDataset
        = vrtwo::PathToOriginalDataset::relativeSymlink;
    vrtwo::generate(fs::absolute(srcPath), root / ovrPathLocal, config);

    // make a link to the vrtwo dataset
    const auto datasetPath(root / name);
    fs::remove(datasetPath);
    fs::create_symlink(ovrPathLocal / "dataset", datasetPath);
}

fs::path vrtWOPath(const calipers::Measurement &cm, const fs::path &rootDir)
{
    switch (cm.datasetType) {
    case calipers::DatasetType::dem:
        return rootDir / "dem";

    case calipers::DatasetType::ophoto:
        return rootDir / "ophoto";
    }

    LOGTHROW(err3, std::logic_error)
        << "Unhandled dataset type <" << cm.datasetType << ">.";
    throw;
}

fs::path createVrtWO(const calipers::Measurement &cm
                     , const fs::path &datasetPath
                     , const fs::path &rootDir
                     , const Config &config)
{
    LOG(info4) << "Generating dataset overviews.";
    LogLinePrefix linePrefix(" (ovr)");

    switch (cm.datasetType) {
    case calipers::DatasetType::dem:
        LOG(info4) << "Generating height overviews.";
        {
            LogLinePrefix linePrefix(" (dem)");
            createVrtWO(datasetPath, rootDir, "dem"
                        , geo::GeoDataset::Resampling::dem, cm);
        }

        LOG(info4) << "Generating minimum height overviews.";
        {
            LogLinePrefix linePrefix(" (min)");
            createVrtWO(datasetPath, rootDir, "dem.min"
                        , geo::GeoDataset::Resampling::minimum, cm);
        }

        LOG(info4) << "Generating maximum height overviews.";
        {
            LogLinePrefix linePrefix(" (max)");
            createVrtWO(datasetPath, rootDir, "dem.max"
                        , geo::GeoDataset::Resampling::maximum, cm);
        }
        break;

    case calipers::DatasetType::ophoto:
        {
            LogLinePrefix linePrefix(" (ophoto)");
            createVrtWO(datasetPath, rootDir, "ophoto"
                        , (config.tmsResampling ? *config.tmsResampling
                           : geo::GeoDataset::Resampling::texture)
                        , cm);
        }
        break;

    default:
        LOGTHROW(err3, std::logic_error)
            << "Unhandled dataset type <" << cm.datasetType << ">.";
    }

    // fails if dataset type is not handled
    return vrtWOPath(cm, rootDir);
}

void buildDefinition(resource::SurfaceDem &def, const calipers::Measurement &cm
                     , const fs::path &dataset, const Config &config)
{
    def.dem.dataset = dataset.string();
    def.dem.geoidGrid = config.geoidGrid;

    def.introspection.position = cm.position;
}

void buildDefinition(resource::TmsRaster &def, const calipers::Measurement&
                     , const fs::path &dataset, const Config &config)
{
    def.dataset = dataset.string();
    def.format = config.format;
    def.resampling = config.tmsResampling;
    def.transparent = config.transparent;
}

template <typename Definition>
void buildDefinition(Resource &r, const calipers::Measurement &cm
                     , const fs::path &dataset
                     , const Config &config)
{
    r.generator = Resource::Generator::from<Definition>();
    auto definition(std::make_shared<Definition>());
    r.definition(definition);
    buildDefinition(*definition, cm, dataset, config);
}

void buildDefinition(Resource &r, const calipers::Measurement &cm
                     , const fs::path &dataset, const Config &config)
{
    switch (cm.datasetType) {
    case calipers::DatasetType::dem:
        buildDefinition<resource::SurfaceDem>(r, cm, dataset, config);
        break;

    case calipers::DatasetType::ophoto:
        buildDefinition<resource::TmsRaster>(r, cm, dataset, config);
        break;
    }
}

void addCredits(Resource &r, const vr::Credit::dict &credits)
{
    for (const auto &credit : credits) {
        r.credits.insert(DualId(credit.id, credit.numericId));
    }
    r.registry.credits.update(credits);
}

std::string md5sum(const fs::path &path)
{
    utility::ifstreambuf f(path.string());
    f.exceptions(std::ios::badbit);

    utility::md5::Md5Sum md5;
    char buf[1<<16];
    std::size_t total(0);
    while (f.readsome(buf, sizeof(buf))) {
        md5.append(buf, f.gcount());
        total += f.gcount();
    }
    return md5.hash();
}

int SetupResource::run()
{
    // find reference frame
    const auto *rf(vr::system.referenceFrames
                   (resourceId_.referenceFrame, std::nothrow));
    if (!rf) {
        LOG(fatal)
            << "There is no reference frame with ID <"
            << resourceId_.referenceFrame << ">.";
        return EXIT_FAILURE;
    }

    // copy config and update
    auto config(config_);
    if (!config.geoidGrid && rf->body) {
        const auto &body(vr::system.bodies(*rf->body));
        config.geoidGrid = body.defaultGeoidGrid;
    }

    // open dataset
    const auto ds(geo::GeoDataset::open(dataset_));

    // 1) deduce resource ID
    const auto resourceId(deduceResourceId(dataset_, resourceId_));
    LOG(info4) << "Using resource ID <" << resourceId << ">.";


    // 2) check resource existence in mapproxy
    LOG(info4) << "Checking for resource existence.";
    {
        Mapproxy mp(config_.mapproxyCtrl);
        if (!mp.supportsReferenceFrame(resourceId_.referenceFrame)) {
            LOG(fatal)
                << "Given reference frame is not supporte by mapproxy. "
                "Please, check that mapproxy uses the same registry and "
                "restart it.";
            return EXIT_FAILURE;
        }

        if (mp.has(resourceId)) {
            LOG(fatal)
                << "Resource " << resourceId << " already exists in mapproxy "
                << "configuration. Please, use another id and/or group.";
            return EXIT_FAILURE;
        }
    }

    // 3) measure dataset
    LOG(info4) << "Measuring dataset.";

    auto cm([&]() -> calipers::Measurement {
        LogLinePrefix linePrefix(" (calipers)");
        calipers::Config calipersConfig;
        calipersConfig.datasetType = asDatasetType(resourceType_);
        return calipers::measure(*rf, ds.descriptor(), calipersConfig);
    }());

    if (cm.nodes.empty()) {
        LOG(fatal)
            << "Unable to set up a mapproxy resource <" << resourceId
            << "> from " << dataset_ << ".";
        return EXIT_FAILURE;
    }

    if (cm.xOverlap) {
        LOG(info3)
            << "Dataset covers whole globe horizontally with "
            << "overlap of " << *cm.xOverlap << " pixels.";
    }

    // 4) allocate attributions
    const auto credits(attributions2credits(config_));

    // 5) copy dataset; TODO: add symlink option
    const auto datasetFileName(fs::path(dataset_).filename());

    const auto datasetHome
        (utility::addExtension(datasetFileName
                               , "." + md5sum(dataset_)));

    const auto rootDir(config_.mapproxyDataRoot / datasetHome);
    const auto baseDatasetPath("original-dataset" / datasetFileName);
    const auto datasetPath(rootDir / baseDatasetPath);

    const auto mainDataset([&]()
    {
        if (!fs::exists(rootDir)) {
            LOG(info4) << "Copying dataset to destination.";

            const auto tmpRootDir(utility::addExtension(rootDir, ".tmp"));

            const auto tmpDatasetPath(tmpRootDir / baseDatasetPath);

            fs::create_directories(tmpDatasetPath.parent_path());

            // copy (overwrite)
            utility::copy_file(dataset_, tmpDatasetPath, true);
            ds.copyFiles(datasetPath);

            // 6) create vrtwo derived datasets
            createVrtWO(cm, tmpDatasetPath , tmpRootDir, config_);

            // commit
            fs::rename(tmpRootDir, rootDir);
        } else {
            LOG(info4) << "Reusing existing dataset.";
        }
        return vrtWOPath(cm, rootDir);
    }());

    // 7) generate tiling information
    LOG(info4) << "Generating tiling information.";
    {
        LogLinePrefix linePrefix(" (tiling)");
        tiling::Config tilingConfig;
        auto ti(tiling::generate(mainDataset, *rf, cm.lodRange
                                 , cm.lodTileRanges(), tilingConfig));

        ti.save(rootDir / ("tiling." + resourceId_.referenceFrame));
    }

    // 8) generate mapproxy resource configuration
    {
        const auto resourceConfigPath
            (config_.mapproxyDefinitionDir / resourceId.group
             / (resourceId.id + "." + resourceId_.referenceFrame + ".json"));
        LOG(info4) << "Generating mapproxy resource configuration at "
                   << resourceConfigPath << ".";

        Resource r({});
        r.id = resourceId;
        r.comment = utility::format
            ("Automatically generated from %s raster dataset."
             , datasetPath.filename());

        // add credits
        addCredits(r, credits);

        r.lodRange = cm.lodRange;
        r.tileRange = cm.tileRange;

        buildDefinition(r, cm, datasetHome, config);

        fs::create_directories(resourceConfigPath.parent_path());
        save(resourceConfigPath, r);

        // check whether there is a group include file
        const auto groupConfigPath
            (config_.mapproxyDefinitionDir / (resourceId.group + ".json"));
        if (!fs::exists(groupConfigPath)) {
            // no -> create
            const auto path(utility::format("%s/*.json", resourceId.group));
            saveIncludeConfig(groupConfigPath, { path });
        }
    }

    // 9) notify mapproxy
    LOG(info4) << "Notifying mapproxy.";
    {
        Mapproxy mp(config_.mapproxyCtrl);

        const auto timestamp(mp.updateResources());
        LOG(info4)
            << "Mapproxy notified. Waiting for update confirmation.";

        const auto waitForUpdate([&](int tries) -> bool
        {
            for (; tries > 0; --tries) {
                if (mp.updatedSince(timestamp)) { return true; }
                usleep(500000);
            }

            return false;
        });

        const auto ready([&](int tries) -> bool
        {
            for (; tries > 0; --tries) {
                if (mp.isReady(resourceId)) {
                    return true;
                }
                usleep(500000);
            }

            return false;
        });

        if (!waitForUpdate(10)) {
            LOG(err3) << "Mapproxy didn't update resources in time. "
                "Check resource presence manually.";
            return EXIT_FAILURE;
        }
        LOG(info3) << "Mapproxy updated resources.";

        if (!ready(10)) {
            LOG(err3) << "Resource has not been made ready in time. "
                "Check resource presence manually.";
            return EXIT_FAILURE;
        }

        const auto url(mp.url(resourceId));

        LOG(info4)
            << "Resource <" << resourceId << "> is ready to serve "
            << "at local URL <" << url << ">.";
    }

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    // force VRT not to share underlying datasets
    geo::Gdal::setOption("VRT_SHARED_SOURCE", 0);
    geo::Gdal::setOption("GDAL_TIFF_INTERNAL_MASK", "YES");
    gdal_drivers::registerAll();
    return SetupResource()(argc, argv);
}
