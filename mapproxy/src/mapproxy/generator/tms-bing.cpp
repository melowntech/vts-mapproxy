#include <future>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"

#include "geo/geodataset.hpp"

#include "imgproc/rastermask/cvmat.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/vts/io.hpp"
#include "vts-libs/vts/nodeinfo.hpp"

#include "../error.hpp"
#include "../support/metatile.hpp"

#include "./tms-bing.hpp"
#include "./factory.hpp"
#include "../support/python.hpp"

#include "browser2d/index.html.hpp"

namespace ba = boost::algorithm;

namespace vr = vadstena::registry;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Params &params)
    {
        return std::make_shared<TmsBing>(params);
    }

    virtual DefinitionBase::pointer definition() {
        return std::make_shared<TmsBing::Definition>();
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (Resource::Generator(Resource::Generator::Type::tms, "tms-bing")
         , std::make_shared<Factory>());
});

void parseDefinition(TmsBing::Definition &def
                     , const Json::Value &value)
{
    std::string s;

    Json::get(def.metadataUrl, value, "metadataUrl");
}

void buildDefinition(Json::Value &value
                     , const TmsBing::Definition &def)
{
    value["metadataUrl"] = def.metadataUrl;
}

void parseDefinition(TmsBing::Definition &def
                     , const boost::python::dict &value)
{
    def.metadataUrl = py2utf8(value["metadataUrl"]);
}

} // namespace

void TmsBing::Definition::from_impl(const boost::any &value)
{
    if (const auto *json = boost::any_cast<Json::Value>(&value)) {
        parseDefinition(*this, *json);
    } else if (const auto *py
               = boost::any_cast<boost::python::dict>(&value))
    {
        parseDefinition(*this, *py);
    } else {
        LOGTHROW(err1, Error)
            << "TmsBing: Unsupported configuration from: <"
            << value.type().name() << ">.";
    }
}

void TmsBing::Definition::to_impl(boost::any &value) const
{
    if (auto *json = boost::any_cast<Json::Value>(&value)) {
        buildDefinition(*json, *this);
    } else {
        LOGTHROW(err1, Error)
            << "TmsBing:: Unsupported serialization into: <"
            << value.type().name() << ">.";
    }
}

bool TmsBing::Definition::operator==(const Definition&) const
{
    // ignore metadata URL, we it has no effect on this resource
    return true;
}

TmsBing::TmsBing(const Params &params)
    : Generator(params)
    , definition_(resource().definition<Definition>())
{
    LOG(info1) << "Generator for <" << id() << "> not ready.";
}

void TmsBing::prepare_impl(Arsenal&)
{
    LOG(info2) << "Preparing <" << id() << ">.";
    makeReady();
}

namespace {

template <typename Callback>
void generateTileUrl(Sink sink, Arsenal &arsenal
                     , const std::string &metadataUrl
                     , Callback callback)
{
    typedef utility::ResourceFetcher::Query Query;
    typedef utility::ResourceFetcher::MultiQuery MultiQuery;

    Query q(metadataUrl);
    q.reuse(false);

    arsenal.fetcher.perform(q, [=](const MultiQuery &query) mutable -> void
    {
        try {
            Json::Value reply;
            Json::Reader reader;
            std::istringstream in(query.front().get().data);
            if (!reader.parse(in, reply)) {
                LOGTHROW(err1, InternalError)
                    << "Unable to parse metadata received from Bing service.";
            }
            const auto &resource
                (reply["resourceSets"][0]["resources"][0]);

            const auto &jurl(resource["imageUrl"]);
            if (!jurl.isString()) {
                LOGTHROW(err1, InternalError)
                    << "Cannot find imageUrl in bind metadata reply.";
            }

            const auto jsubdomains(resource["imageUrlSubdomains"]);
            if (!jsubdomains.isArray()) {
                LOGTHROW(err1, InternalError)
                    << "Cannot find imageUrl in bind metadata reply.";
            }

            auto subdomains([&]() -> std::string
            {
                std::string res("{alt(");
                bool first(true);
                for (const auto &js : jsubdomains) {
                    if (first) {
                        first = false;
                    } else {
                        res.push_back(',');
                    }
                    res.append(js.asString());
                }
                res.append(")}");
                return res;
            });

            auto url(jurl.asString());

            ba::replace_all(url, "{quadkey}", "{quad(loclod,locx,locy)}");
            ba::replace_all(url, "{subdomain}", subdomains());

            callback(sink, url);
        } catch (...) {
            sink.error();
        }
    });
}

} // namespace

vr::BoundLayer TmsBing::boundLayer(ResourceRoot, const std::string &url)
    const
{
    const auto &res(resource());

    vr::BoundLayer bl;
    bl.id = res.id.fullId();
    bl.numericId = 0; // no numeric ID
    bl.type = vr::BoundLayer::Type::raster;

    // build url
    bl.url = url;

    bl.lodRange = res.lodRange;
    bl.tileRange = res.tileRange;
    bl.credits = asInlineCredits(res.credits);

    // availability: available images are all images except PNGs
    bl.availability = boost::in_place();
    bl.availability->type = vr::BoundLayer::Availability::Type::negativeType;
    bl.availability->mime = "image/png";

    // done
    return bl;
}

vts::MapConfig TmsBing::mapConfig_impl(ResourceRoot root)
    const
{
    const auto &res(resource());

    vts::MapConfig mapConfig;
    mapConfig.referenceFrame = *res.referenceFrame;

    // this is Tiled service: we have bound layer only; use remote definition
    mapConfig.boundLayers.add
        (vr::BoundLayer
         (res.id.fullId()
          , prependRoot(std::string("boundlayer.json"), resource(), root)));

    return mapConfig;
}

Generator::Task TmsBing::generateFile_impl(const FileInfo &fileInfo
                                           , Sink &sink) const
{
    TmsFileInfo fi(fileInfo);

    switch (fi.type) {
    case TmsFileInfo::Type::unknown:
        sink.error(utility::makeError<NotFound>("Unrecognized filename."));
        break;

    case TmsFileInfo::Type::config: {
        std::ostringstream os;
        mapConfig(os, ResourceRoot::none);
        sink.content(os.str(), fi.sinkFileInfo());
        break;
    }

    case TmsFileInfo::Type::definition:
        return [this, fi](Sink &sink, Arsenal &arsenal) {
            generateTileUrl(sink, arsenal, definition_.metadataUrl
                            , [this, fi](Sink sink, const std::string &url)
                            mutable
            {
                std::ostringstream os;
                vr::saveBoundLayer(os, boundLayer(ResourceRoot::none, url));
                sink.content(os.str(), fi.sinkFileInfo());
            });
        };

    case TmsFileInfo::Type::support:
        sink.content(fi.support->data, fi.support->size
                      , fi.sinkFileInfo(), false);
        break;

    case TmsFileInfo::Type::image: {
        sink.error(utility::makeError<NotFound>
                    ("Remote tms driver is unable to generate any image."));
        return {};
    }

    case TmsFileInfo::Type::mask:
        sink.error(utility::makeError<NotFound>
                    ("Bing tms driver is unable to generate any metatile."));
        return {};

    case TmsFileInfo::Type::metatile:
        sink.error(utility::makeError<NotFound>
                    ("Bing tms driver is unable to generate any mask."));
        return {};
    }

    return {};
}

} // namespace generator
