#include "utility/raise.hpp"

#include "vts-libs/vts/mapconfig.hpp"
#include "vts-libs/registry.hpp"

#include "./fileinfo.hpp"
#include "./error.hpp"
#include "./core.hpp"

namespace vts = vadstena::vts;
namespace vr = vadstena::registry;

struct Core::Detail : boost::noncopyable {
    Detail(Generators &generators)
        : generators(generators)
        , browserEnabled(generators.config().fileFlags
                         & FileFlags::browserEnabled)
    {}

    void generate(const std::string &location
                  , const Sink::pointer &sink);

    void generateRfMapConfig(const std::string &referenceFrame
                             , const Sink::pointer &sink);

    void generateResourceFile(const FileInfo &fi
                              , const Sink::pointer &sink);

    void generateListing(const FileInfo &fi
                         , const Sink::pointer &sink);

    bool assertBrowserEnabled(const Sink::pointer &sink) const {
        if (browserEnabled) { return true; }
        sink->error(utility::makeError<NotFound>("Browsing disabled."));
        return false;
    }

    Generators &generators;
    bool browserEnabled;
};

Core::Core(Generators &generators)
    : detail_(std::make_shared<Detail>(generators))
{}

void Core::generate_impl(const std::string &location
                         , const Sink::pointer &sink)
{
    detail().generate(location, sink);
}

namespace {

const std::string& getItem(const std::string &s)
{
    return s;
}

template <typename T>
const std::string& getName(const std::pair<std::string, T> &p)
{
    return p.first;
}

template <typename T>
const std::string& getName(const std::pair<const std::string, T> &p)
{
    return p.first;
}

template <typename T>
std::string getName(const T &value)
{
    return boost::lexical_cast<std::string>(value);
}

template <typename Container>
std::vector<std::string> buildListing(const Container &container)
{
    std::vector<std::string> out;
    for (const auto &item : container) {
        out.push_back(getName(item));
    }
    return out;
}

} // namespace

void Core::Detail::generate(const std::string &location
                            , const Sink::pointer &sink)
{
    try {
        FileInfo fi(location);

        switch (fi.type) {
        case FileInfo::Type::referenceFrameMapConfig:
            generateRfMapConfig(fi.resourceId.referenceFrame, sink);
            return;

        case FileInfo::Type::resourceFile:
            generateResourceFile(fi, sink);
            return;

        case FileInfo::Type::dirRedir:
            // append slash to filename and let browser try luck again
            sink->seeOther(fi.filename + "/");
            return;

        case FileInfo::Type::referenceFrameListing:
        case FileInfo::Type::typeListing:
        case FileInfo::Type::groupListing:
        case FileInfo::Type::idListing:
            generateListing(fi, sink);
            return;

        default: break;
        }

        sink->error(utility::makeError<InternalError>("Unhandled."));
    } catch (...) {
        sink->error();
    }
}

void Core::Detail::generateRfMapConfig(const std::string &referenceFrame
                                       , const Sink::pointer &sink)
{
    auto genlist(generators.referenceFrame(referenceFrame));
    if (genlist.empty()) {
        sink->error(utility::makeError<NotFound>
                    ("No data for <%s>.", referenceFrame));
    }

    // build map
    vts::MapConfig mapConfig;
    for (const auto &generator : genlist) {
        mapConfig.merge(generator->mapConfig(ResourceRoot::type));
    }

    std::ostringstream os;
    vts::saveMapConfig(mapConfig, os);
    sink->content(os.str(), Sink::FileInfo("application/json"));
}

void Core::Detail::generateResourceFile(const FileInfo &fi
                                        , const Sink::pointer &sink)
{
    auto generator(generators.generator(fi));
    if (!generator) {
        sink->error(utility::makeError<NotFound>
                    ("No generator for URL <%s> found.", fi.url));
        return;
    }

    if (auto task = generator->generateFile(fi, sink)) {
        // TODO: enqueue task
        (void) task;
    }
}

void Core::Detail::generateListing(const FileInfo &fi
                                   , const Sink::pointer &sink)
{
    if (!assertBrowserEnabled(sink)) { return; }

    switch (fi.type) {
    case FileInfo::Type::referenceFrameListing:
        sink->listing(buildListing(vr::Registry::referenceFrames()));
        return;

    case FileInfo::Type::typeListing:
        sink->listing
            (buildListing(enumerationValues
                          (Resource::Generator::Type())));
        return;

    case FileInfo::Type::groupListing:
        sink->listing(buildListing
                      (generators.listGroups
                       (fi.resourceId.referenceFrame, fi.generatorType)));
        return;

    case FileInfo::Type::idListing:
        sink->listing(buildListing
                      (generators.listIds
                       (fi.resourceId.referenceFrame, fi.generatorType
                        , fi.resourceId.group)));
        return;

    default: break;
    }

    sink->error(utility::makeError<InternalError>("Unhandled."));
}
