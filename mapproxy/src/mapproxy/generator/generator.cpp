#include <thread>
#include <condition_variable>

#include "dbglog/dbglog.hpp"

#include "../error.hpp"
#include "../generator.hpp"
#include "./factory.hpp"

namespace fs = boost::filesystem;

namespace {

typedef std::map<Resource::Generator, Generator::Factory::pointer> Registry;
Registry registry;

Generator::Factory::pointer findFactory(const Resource::Generator &type)
{
    auto fregistry(registry.find(type));
    if (fregistry == registry.end()) {
        LOGTHROW(err1, UnknownGenerator)
            << "Unknown generator type <" << type << ">.";
    }
    return fregistry->second;
}

} // namespace

void Generator::registerType(const Resource::Generator &type
                             , const Factory::pointer &factory)
{
    registry.insert(Registry::value_type(type, factory));
}

Generator::pointer Generator::create(const Resource::Generator &type
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

void Generator::makeReady()
{
    ready_ = true;
    LOG(info2) << "Ready to serve resource <" << id()
               << "> (type <" << resource().generator << ">).";
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
    void update(const Resource::map &resources);

    fs::path makePath(const Resource::Id &id);

    void runUpdater();

    const boost::filesystem::path root_;
    ResourceBackend::pointer resourceBackend_;
    const int resourceUpdatePeriod_;

    // resource updater stuff
    std::thread updater_;
    std::atomic<bool> updaterRunning_;
    std::mutex updaterLock_;
    std::condition_variable updaterCond_;

    // internals
    std::mutex servingLock_;
    Generator::map serving_;

    std::mutex preparingLock_;
    Generator::list preparing_;
    std::condition_variable preparingCond_;
};

fs::path Generators::Detail::makePath(const Resource::Id &id)
{
    return (root_ / id.group / id.id);
}

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

void Generators::Detail::update(const Resource::map &resources)
{
    LOG(info2) << "Updating resources.";

    auto iresources(resources.begin()), eresources(resources.end());
    auto iserving(serving_.begin()), eserving(serving_.end());

    Generator::list toAdd;
    Generator::list toRemove;

    auto add([&](const Resource &res)
    {
        try {
            toAdd.push_back(Generator::create
                            (res.generator, makePath(res.id), res));
        } catch (const std::exception &e) {
            LOG(err2) << "Failed to create generator for resource <"
                      << iresources->first << ">: <" << e.what() << ">.";
        }
    });

    // process common stuff
    while ((iresources != eresources) && (iserving != eserving)) {
        if (iresources->first < iserving->first) {
            // new resource
            add(iresources->second);
            ++iresources;
        } else if (iserving->first < iresources->first) {
            // removed resource
            // TODO: remove
            LOG(info2) << "removed: " << iserving->first;
            toRemove.push_back(iserving->second);
            ++iserving;
        } else {
            // existing resource
            // TODO: check for change
            LOG(info2) << "reused: " << iresources->first;
            ++iresources;
            ++iserving;
        }
    }

    // process tail: added resources
    for (; iresources != eresources; ++iresources) {
        add(iresources->second);
    }

    // process tail: removed resources
    for (; iserving != eserving; ++iserving) {
        toRemove.push_back(iserving->second);
    }

    // add stuff
    for (const auto &generator : toAdd) {
        {
            std::unique_lock<std::mutex> lock(servingLock_);
            serving_.insert(Generator::map::value_type
                            (generator->id(), generator));
        }

        if (!generator->ready()) {
            std::unique_lock<std::mutex> lock(preparingLock_);
            preparing_.push_back(generator);
            preparingCond_.notify_one();
        }
    }

    // TODO: process remove set

    LOG(info2) << "Resources updated.";
}
