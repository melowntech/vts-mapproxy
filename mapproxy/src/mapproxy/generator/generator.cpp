#include <thread>
#include <condition_variable>

#include "dbglog/dbglog.hpp"

#include "../error.hpp"
#include "../generator.hpp"
#include "./factory.hpp"

namespace {

typedef std::map<std::string, Generator::Factory::pointer> Registry;
Registry registry;

Generator::Factory::pointer findFactory(const std::string &type)
{
    auto fregistry(registry.find(type));
    if (fregistry == registry.end()) {
        LOGTHROW(err1, UnknownGenerator)
            << "Unknown resource backend <" << type << ">.";
    }
    return fregistry->second;
}

} // namespace

void Generator::registerType(const std::string &type
                             , const Factory::pointer &factory)
{
    registry.insert(Registry::value_type(type, factory));
}

Generator::pointer Generator::create(const std::string &type
                                     , const boost::filesystem::path &root
                                     , const Resource &resource)
{
    try {
        return findFactory(type)->create(root, resource);
    } catch (const boost::bad_any_cast&) {
        LOGTHROW(err2, InvalidConfiguration)
            << "Passed resource does not match generator <"
            << type << ">.";
    }
    throw;
}

class Generators::Detail {
public:
    Detail(const boost::filesystem::path &root
           , const ResourceBackend::pointer &resourceBackend
           , int resourceUpdatePeriod)
        : root_(root), resourceBackend_(resourceBackend)
        , resourceUpdatePeriod_(resourceUpdatePeriod)
        , updaterRunning_(false)
    {}

    Generator::pointer matchUrl(const std::string &path) const;

    Generator::list referenceFrame(const std::string &referenceFrame) const;

    void start();
    void stop();

private:
    void update(const Resource::Groups &resources);

    void runUpdater();

    const boost::filesystem::path root_;
    ResourceBackend::pointer resourceBackend_;
    const int resourceUpdatePeriod_;

    std::thread updater_;
    std::atomic<bool> updaterRunning_;
    std::mutex updaterLock_;
    std::condition_variable updaterCond_;
};

void Generators::Detail::start()
{
    // start updater
    updaterRunning_ = true;
    std::thread updater(&Detail::runUpdater, this);
    updater_.swap(updater);
}

void Generators::Detail::stop()
{
    if (updaterRunning_) {
        updaterRunning_ = false;
        updaterCond_.notify_all();
        updater_.join();
    }
}

void Generators::Detail::runUpdater()
{
    dbglog::thread_id("updater");


    while (updaterRunning_) {
        std::chrono::seconds sleep(resourceUpdatePeriod_);

        try {
            update(resourceBackend_->load());
        } catch (const std::exception &e) {
            LOG(err2) << "Resource info update failed: <" << e.what() << ">.";
            sleep = std::chrono::seconds(5);
        }

        // sleep for 5 minutes
        {
            std::unique_lock<std::mutex> lock(updaterLock_);
            updaterCond_.wait_for(lock, sleep);
        }
    }
}

Generators::Generators(const boost::filesystem::path &root
                       , const ResourceBackend::pointer &resourceBackend
                       , int resourceUpdatePeriod)
    : detail_(std::make_shared<Detail>
              (root, resourceBackend, resourceUpdatePeriod))
{
    detail().start();
}

Generators::~Generators()
{
    detail().stop();
}

Generator::pointer Generators::matchUrl(const std::string &path)
{
    return detail().matchUrl(path);
}

Generator::list Generators::referenceFrame(const std::string &referenceFrame)
{
    return detail().referenceFrame(referenceFrame);
}

Generator::pointer Generators::Detail::matchUrl(const std::string &path)
    const
{
    (void) path;
    return {};
}

Generator::list
Generators::Detail::referenceFrame(const std::string &referenceFrame)
    const
{
    (void) referenceFrame;
    return {};
}

void Generators::Detail::update(const Resource::Groups &resources)
{
    LOG(info2) << "Updating resources.";
    (void) resources;
}
