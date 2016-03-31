#include <thread>

#include <boost/format.hpp>
#include <boost/asio.hpp>

#include "utility/raise.hpp"

#include "vts-libs/vts/mapconfig.hpp"
#include "vts-libs/registry.hpp"

#include "./fileinfo.hpp"
#include "./error.hpp"
#include "./core.hpp"

namespace asio = boost::asio;
namespace vts = vadstena::vts;
namespace vr = vadstena::registry;

class Core::Detail : boost::noncopyable {
public:
    Detail(Generators &generators)
        : generators_(generators)
        , browserEnabled_(generators.config().fileFlags
                          & FileFlags::browserEnabled)
        , work_(ios_)
    {
        // TODO: make configurable
        start(5);
    }

    ~Detail() {
        stop();
    }

    void generate(const std::string &location
                  , const Sink::pointer &sink);

    void generateRfMapConfig(const std::string &referenceFrame
                             , const Sink::pointer &sink);

    void generateResourceFile(const FileInfo &fi
                              , const Sink::pointer &sink);

    void generateListing(const FileInfo &fi
                         , const Sink::pointer &sink);

    bool assertBrowserEnabled(const Sink::pointer &sink) const {
        if (browserEnabled_) { return true; }
        sink->error(utility::makeError<NotFound>("Browsing disabled."));
        return false;
    }

private:
    void start(std::size_t count);
    void stop();
    void worker(std::size_t id);
    void post(const Generator::Task &task
              , const Sink::pointer &sink);

    Generators &generators_;
    bool browserEnabled_;

    /** Processing pool stuff.
     */
    asio::io_service ios_;
    asio::io_service::work work_;
    std::vector<std::thread> workers_;
};

void Core::Detail::start(std::size_t count)
{
    // make sure threads are released when something goes wrong
    struct Guard {
        Guard(const std::function<void()> &func) : func(func) {}
        ~Guard() { if (func) { func(); } }
        void release() { func = {}; }
        std::function<void()> func;
    } guard(std::bind(&Detail::stop, this));

    for (std::size_t id(1); id <= count; ++id) {
        workers_.emplace_back(&Detail::worker, this, id);
    }

    guard.release();
}

void Core::Detail::stop()
{
    ios_.stop();

    while (!workers_.empty()) {
        workers_.back().join();
        workers_.pop_back();
    }
}

void Core::Detail::worker(std::size_t id)
{
    dbglog::thread_id(str(boost::format("worker:%u") % id));
    LOG(info2) << "Spawned worker id:" << id << ".";

    for (;;) {
        try {
            ios_.run();
            LOG(info2) << "Terminated worker id:" << id << ".";
            return;
        } catch (const std::exception &e) {
            LOG(err3)
                << "Uncaught exception in worker: <" << e.what()
                << ">. Going on.";
        }
    }

}

void Core::Detail::post(const Generator::Task &task
                        , const Sink::pointer &sink)
{
    if (!task) { return; }

    ios_.post([=]()
    {
        try {
            task();
        } catch (...) {
            sink->error();
        }
    });
}

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
Sink::Listing buildListing(const Container &container
                           , const Sink::Listing &bootstrap = Sink::Listing())
{
    Sink::Listing out(bootstrap);

    for (const auto &item : container) {
        out.emplace_back(getName(item), Sink::ListingItem::Type::dir);
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
    auto genlist(generators_.referenceFrame(referenceFrame));
    if (genlist.empty()) {
        sink->error(utility::makeError<NotFound>
                    ("No data for <%s>.", referenceFrame));
        return;
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
    auto generator(generators_.generator(fi));
    if (!generator) {
        sink->error(utility::makeError<NotFound>
                    ("No generator for URL <%s> found.", fi.url));
        return;
    }

    post(generator->generateFile(fi, sink), sink);
}

namespace {

Sink::Listing browsableDirectoryContent = {
    { "index.html" }
    , { "mapConfig.json" }
};

Sink::Listing otherDirectoryContent = {
    { "index.html" }
};

} // namespace

void Core::Detail::generateListing(const FileInfo &fi
                                   , const Sink::pointer &sink)
{
    if (!assertBrowserEnabled(sink)) { return; }

    switch (fi.type) {
    case FileInfo::Type::referenceFrameListing:
        sink->listing(buildListing(vr::Registry::referenceFrames()
                                   , otherDirectoryContent));
        return;

    case FileInfo::Type::typeListing:
        sink->listing
            (buildListing(enumerationValues(Resource::Generator::Type())
                          , browsableDirectoryContent));
        return;

    case FileInfo::Type::groupListing:
        sink->listing(buildListing
                      (generators_.listGroups
                       (fi.resourceId.referenceFrame, fi.generatorType)
                       , browsableDirectoryContent));
        return;

    case FileInfo::Type::idListing:
        sink->listing(buildListing
                      (generators_.listIds
                       (fi.resourceId.referenceFrame, fi.generatorType
                        , fi.resourceId.group)
                       , browsableDirectoryContent));
        return;

    default: break;
    }

    sink->error(utility::makeError<InternalError>("Unhandled."));
}
