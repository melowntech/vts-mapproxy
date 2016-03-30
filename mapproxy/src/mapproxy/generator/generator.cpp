#include <thread>
#include <condition_variable>
#include <sstream>

#include "dbglog/dbglog.hpp"

#include "../error.hpp"
#include "../generator.hpp"
#include "./factory.hpp"

namespace fs = boost::filesystem;

namespace {

const std::string ResourceFile("resource.json");

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

Generator::pointer Generator::create(const Config &config
                                     , const Resource::Generator &type
                                     , const Resource &resource)
{
    try {
        return findFactory(type)->create(config, resource);
    } catch (const boost::bad_any_cast&) {
        LOGTHROW(err2, InvalidConfiguration)
            << "Passed resource does not match generator <"
            << type << ">.";
    }
    throw;
}

Generator::Generator(const Config &config
                     , const Resource &resource)
    : config_(config), resource_(resource), savedResource_(resource)
    , fresh_(false), ready_(false)
{
    // TODO: handle failed creation
    auto rfile(root() / ResourceFile);

    if (create_directories(root())) {
        // new resource
        fresh_ = true;
        save(rfile, resource);
    } else {
        // reopen of existing dataset
        savedResource_ = loadResource(rfile);
        if (savedResource_ != resource) {
            LOG(warn3)
                << "Definition of resource <" << resource.id
                << "> differs from the one stored in store at "
                << root() << "; using stored definition.";
            resource_ = savedResource_;
        }
    }
}

bool Generator::check(const Resource &resource) const
{
    if (resource_ != resource) {
        LOG(warn2)
            << "Definition of resource <" << resource.id
            << "> differs from the one stored in store at "
            << root() << "; using stored definition.";
        return false;
    }
    return true;
}

void Generator::makeReady()
{
    ready_ = true;
    LOG(info2) << "Ready to serve resource <" << id()
               << "> (type <" << resource().generator << ">).";
}

class Generators::Detail {
public:
    Detail(const Generators::Config &config
           , const ResourceBackend::pointer &resourceBackend)
        : config_(config), resourceBackend_(resourceBackend)
        , updaterRunning_(false)
    {}

    Generator::pointer generator(const FileInfo &fileInfo) const;

    Generator::list referenceFrame(const std::string &referenceFrame) const;

    void start();
    void stop();

private:
    void update(const Resource::map &resources);

    fs::path makePath(const Resource::Id &id);

    void runUpdater();

    const Config config_;
    ResourceBackend::pointer resourceBackend_;

    // resource updater stuff
    std::thread updater_;
    std::atomic<bool> updaterRunning_;
    std::mutex updaterLock_;
    std::condition_variable updaterCond_;

    // internals
    mutable std::mutex servingLock_;
    Generator::map serving_;

    std::mutex preparingLock_;
    Generator::list preparing_;
    std::condition_variable preparingCond_;
};

fs::path Generators::Detail::makePath(const Resource::Id &id)
{
    return (config_.root / id.group / id.id);
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
        std::chrono::seconds sleep(config_.resourceUpdatePeriod);

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

Generators::Generators(const Config &config
                       , const ResourceBackend::pointer &resourceBackend)
    : detail_(std::make_shared<Detail>(config, resourceBackend))
{
    detail().start();
}

Generators::~Generators()
{
    detail().stop();
}

Generator::list Generators::referenceFrame(const std::string &referenceFrame)
    const
{
    return detail().referenceFrame(referenceFrame);
}

Generator::pointer Generators::generator(const FileInfo &fileInfo)
    const
{
    return detail().generator(fileInfo);
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
            Generator::Config config;
            config.root = makePath(res.id);
            config.fileFlags = config_.fileFlags;
            toAdd.push_back(Generator::create(config, res.generator, res));
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
            toRemove.push_back(iserving->second);
            ++iserving;
        } else {
            // existing resource
            iserving->second->check(iresources->second);
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

        // TODO: better prepare set; probably multi-index container: sequence
        // with additional resource index map
        if (!generator->ready()) {
            std::unique_lock<std::mutex> lock(preparingLock_);
            preparing_.push_back(generator);
            preparingCond_.notify_one();
        }
    }

    // remove stuff
    for (const auto &generator : toRemove) {
        // TODO: remove from prepare set (should be m-i container as mentioned
        // above)
        {
            std::unique_lock<std::mutex> lock(servingLock_);
            serving_.erase(generator->id());
        }
    }

    LOG(info2) << "Resources updated.";
}

Generator::list
Generators::Detail::referenceFrame(const std::string &referenceFrame)
    const
{
    Generator::list out;

    // use only ready generators that handle datasets for given reference frame
    std::unique_lock<std::mutex> lock(servingLock_);
    for (const auto &item: serving_) {
        if (item.second->ready()
            && item.second->handlesReferenceFrame(referenceFrame))
        {
            out.push_back(item.second);
        }
    }

    return out;
}

Generator::pointer Generators::Detail::generator(const FileInfo &fileInfo)
    const
{
    // find generator (under lock)
    auto generator([&]() -> Generator::pointer
    {
        std::unique_lock<std::mutex> lock(servingLock_);
        auto fserving(serving_.find(fileInfo.resourceId));
        if (fserving == serving_.end()) { return {}; }
        return fserving->second;
    }());

    const auto &resource(generator->resource());

    // check reference frame
    if (!resource.referenceFrames.count(fileInfo.referenceFrame)) {
        return {};
    }

    // check generator type
    if (fileInfo.generatorType != resource.generator.type) {
        return {};
    }

    return generator;
}

void Generator::mapConfig(std::ostream &os, const std::string &referenceFrame
                          , ResourceRoot root) const
{
    vts::MapConfig mc(mapConfig(referenceFrame, root));
    vts::saveMapConfig(mc, os);
}
