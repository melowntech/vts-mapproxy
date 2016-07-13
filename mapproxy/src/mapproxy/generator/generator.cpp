#include <thread>
#include <condition_variable>
#include <sstream>
#include <deque>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/asio.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/global_fun.hpp>
#include <boost/multi_index/mem_fun.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/path.hpp"

#include "../error.hpp"
#include "../generator.hpp"
#include "./factory.hpp"

namespace fs = boost::filesystem;
namespace asio = boost::asio;
namespace bmi = boost::multi_index;

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

DefinitionBase::pointer Generator::definition(const Resource::Generator &type)
{
    return findFactory(type)->definition();
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
    config_.root = (config_.root / resource_.id.referenceFrame
                    / resource_.id.group / resource_.id.id);

    // TODO: handle failed creation
    auto rfile(root() / ResourceFile);

    if (create_directories(root())) {
        // new resource
        fresh_ = true;
        save(rfile, resource);
    } else {
        // reopen of existing dataset
        savedResource_ = loadResource(rfile).front();
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

void Generator::mapConfig(std::ostream &os, ResourceRoot root) const
{
    vts::MapConfig mc(mapConfig(root));
    vts::saveMapConfig(mc, os);
}

std::string Generator::absoluteDataset(const std::string &path)
    const
{
    // TODO: handle non-path resources (i.e. URL's)
    return absolute(path, config_.resourceRoot).string();
}

boost::filesystem::path
Generator::absoluteDataset(const boost::filesystem::path &path)
    const
{
    return absolute(path, config_.resourceRoot);
}

boost::optional<std::string>
Generator::absoluteDataset(const boost::optional<std::string> &path) const
{
    if (!path) { return path; }
    return absoluteDataset(*path);
}

std::string Generator
::absoluteDataset(const std::string &path
                  , const boost::optional<std::string> &override) const
{
    if (override) { return absoluteDataset(*override); }
    return absoluteDataset(path);
}

boost::optional<boost::filesystem::path>
Generator
::absoluteDatasetRf(const boost::optional<boost::filesystem::path> &path)
    const
{
    if (!path) { return path; }
    return absoluteDataset
        (utility::addExtension(*path, "." + referenceFrameId()));
}

void Generator::checkReady() const
{
    if (ready_) { return; }
    throw Unavailable("Generator not ready.");
}

namespace {

struct TypeKey {
    std::string referenceFrame;
    Resource::Generator::Type type;

    bool operator<(const TypeKey &o) const {
        if (referenceFrame < o.referenceFrame) { return true; }
        if (o.referenceFrame < referenceFrame) { return false; }
        return type < o.type;
    }

    TypeKey(const std::string &referenceFrame, Resource::Generator::Type type)
        : referenceFrame(referenceFrame), type(type)
    {}
};

TypeKey extractTypeKey(const Generator &generator)
{
    const auto r(generator.resource());
    return { r.id.referenceFrame, r.generator.type };
}

struct GroupKey {
    std::string referenceFrame;
    Resource::Generator::Type type;
    std::string group;

    bool operator<(const GroupKey &o) const {
        if (referenceFrame < o.referenceFrame) { return true; }
        if (o.referenceFrame < referenceFrame) { return false; }

        if (group < o.group) { return true; }
        if (o.group < group) { return false; }

        return type < o.type;
    }

    GroupKey(const std::string &referenceFrame, Resource::Generator::Type type
             , const std::string &group)
        : referenceFrame(referenceFrame), type(type), group(group)
    {}
};

GroupKey extractGroupKey(const Generator &generator)
{
    const auto r(generator.resource());
    return { r.id.referenceFrame, r.generator.type, r.id.group };
}

} // namespace

class Generators::Detail {
public:
    Detail(const Generators::Config &config
           , const ResourceBackend::pointer &resourceBackend)
        : config_(config), resourceBackend_(resourceBackend)
        , running_(false), ready_(false)
        , work_(ios_)
    {}

    void checkReady() const;

    Generator::pointer generator(const FileInfo &fileInfo) const;

    Generator::list referenceFrame(const std::string &referenceFrame) const;

    std::vector<std::string> listGroups(const std::string &referenceFrame
                                        , Resource::Generator::Type type)
        const;

    std::vector<std::string> listIds(const std::string &referenceFrame
                                     , Resource::Generator::Type type
                                     , const std::string &group) const;

    void start();
    void stop();

    inline const Config& config() const { return config_; }

private:
    void update(const Resource::map &resources);

    void updater();
    void worker(std::size_t id);
    void prepare(const Generator::pointer &generator);

    const Config config_;
    ResourceBackend::pointer resourceBackend_;

    // resource updater stuff
    std::thread updater_;
    std::atomic<bool> running_;
    std::mutex updaterLock_;
    std::condition_variable updaterCond_;

    struct ResourceIdIdx {};
    struct GroupIdx {};
    struct TypeIdx {};
    struct ReferenceFrameIdx {};

    typedef boost::multi_index_container<
        Generator::pointer
        , bmi::indexed_by<
              bmi::ordered_unique<bmi::identity<Generator::pointer> >

              , bmi::ordered_unique<
                    bmi::tag<ResourceIdIdx>
                    , BOOST_MULTI_INDEX_CONST_MEM_FUN
                    (Generator, const Resource::Id&, id)
                    >

              , bmi::ordered_non_unique<
                    bmi::tag<TypeIdx>
                    , bmi::global_fun<const Generator&, TypeKey
                                      , &extractTypeKey>
                    >

              , bmi::ordered_non_unique<
                    bmi::tag<GroupIdx>
                    , bmi::global_fun<const Generator&, GroupKey
                                      , &extractGroupKey>
                    >

              , bmi::ordered_non_unique<
                    bmi::tag<ReferenceFrameIdx>
                    , BOOST_MULTI_INDEX_CONST_MEM_FUN
                    (Generator, const std::string&, referenceFrameId)
                    >
              >

        > GeneratorMap;



        // internals
    mutable std::mutex lock_;
    GeneratorMap serving_;

    std::atomic<bool> ready_;

    // prepare stuff
    asio::io_service ios_;
    asio::io_service::work work_;
    std::vector<std::thread> workers_;
};

void Generators::Detail::checkReady() const
{
    if (ready_) { return; }
    throw Unavailable("Server not ready.");
}

void Generators::Detail::start()
{
    // make sure threads are released when something goes wrong
    struct Guard {
        Guard(const std::function<void()> &func) : func(func) {}
        ~Guard() { if (func) { func(); } }
        void release() { func = {}; }
        std::function<void()> func;
    } guard([this]() { stop(); });

    // start updater
    running_ = true;
    std::thread updater(&Detail::updater, this);
    updater_.swap(updater);

    // TODO: make configurable
    std::size_t count(5);
    // start workers
    for (std::size_t id(1); id <= count; ++id) {
        workers_.emplace_back(&Detail::worker, this, id);
    }

    guard.release();
}

void Generators::Detail::stop()
{
    if (!running_) { return; }

    running_ = false;
    ios_.stop();

    updaterCond_.notify_all();
    updater_.join();

    while (!workers_.empty()) {
        workers_.back().join();
        workers_.pop_back();
    }
}

struct Aborted {};

void Generators::Detail::updater()
{
    dbglog::thread_id("updater");


    while (running_) {
        std::chrono::seconds sleep(config_.resourceUpdatePeriod);

        try {
            update(resourceBackend_->load());
        } catch (Aborted) {
            // pass
        } catch (const std::exception &e) {
            LOG(err2) << "Resource info update failed: <" << e.what() << ">.";
            sleep = std::chrono::seconds(5);
        }

        // sleep for 5 minutes
        {
            std::unique_lock<std::mutex> lock(updaterLock_);
            updaterCond_.wait_for(lock, sleep, [this]()
            {
                return !running_;
            });
        }
    }
}

void Generators::Detail::worker(std::size_t id)
{
    dbglog::thread_id(str(boost::format("prepare:%u") % id));
    LOG(info2) << "Spawned prepare worker id:" << id << ".";

    for (;;) {
        try {
            ios_.run();
            LOG(info2) << "Terminated prepare worker id:" << id << ".";
            return;
        } catch (const std::exception &e) {
            LOG(err3)
                << "Uncaught exception in worker: <" << e.what()
                << ">. Going on.";
        }
    }
}

void Generators::Detail::prepare(const Generator::pointer &generator)
{
    ios_.post([=]()
    {
        try {
            generator->prepare();
        } catch (const std::exception &e) {
            LOG(warn2)
                << "Failed to prepare generator for <"
                << generator->resource().id << "> (" << e.what()
                << "); removing from set of known generators.";

            // erease from map (under lock)
            std::unique_lock<std::mutex> lock(lock_);
            serving_.erase(generator);
        }
    });
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
    auto &idx(serving_.get<ResourceIdIdx>());
    auto iserving(idx.begin()), eserving(idx.end());

    Generator::list toAdd;
    Generator::list toRemove;

    auto add([&](const Resource &res)
    {
        if (!running_) {
            throw Aborted{};
        }
        try {
            Generator::Config config(config_);
            config.root = config_.root;
            toAdd.push_back(Generator::create(config, res.generator, res));
        } catch (const std::exception &e) {
            LOG(err2) << "Failed to create generator for resource <"
                      << iresources->first << ">: <" << e.what() << ">.";
        }
    });

    // process common stuff
    while ((iresources != eresources) && (iserving != eserving)) {
        if (iresources->first < (*iserving)->id()) {
            // new resource
            add(iresources->second);
            ++iresources;
        } else if ((*iserving)->id() < iresources->first) {
            // removed resource
            toRemove.push_back(*iserving);
            ++iserving;
        } else {
            // existing resource
            (*iserving)->check(iresources->second);
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
        toRemove.push_back(*iserving);
    }


    // add stuff
    for (const auto &generator : toAdd) {
        {
            std::unique_lock<std::mutex> lock(lock_);
            serving_.insert(generator);
        }

        if (!generator->ready()) {
            prepare(generator);
        }
    }

    // remove stuff
    for (const auto &generator : toRemove) {
        {
            std::unique_lock<std::mutex> lock(lock_);
            serving_.erase(generator);
        }

        // TODO: mark as to be removed for prepare workers
    }

    LOG(info2) << "Resources updated.";
    if (!ready_) {
        ready_ = true;
        LOG(info3) << "Ready to serve.";
    }
}

Generator::list
Generators::Detail::referenceFrame(const std::string &referenceFrame)
    const
{
    checkReady();

    Generator::list out;

    // use only ready generators that handle datasets for given reference frame
    std::unique_lock<std::mutex> lock(lock_);
    auto &idx(serving_.get<ReferenceFrameIdx>());
    for (auto range(idx.equal_range(referenceFrame));
         range.first != range.second; ++range.first)
    {
        if ((*range.first)->ready()) {
            out.push_back(*range.first);
        }
    }

    return out;
}

Generator::pointer Generators::Detail::generator(const FileInfo &fileInfo)
    const
{
    checkReady();

    // find generator (under lock)
    auto generator([&]() -> Generator::pointer
    {
        std::unique_lock<std::mutex> lock(lock_);
        auto &idx(serving_.get<ResourceIdIdx>());
        auto fserving(idx.find(fileInfo.resourceId));
        if (fserving == idx.end()) { return {}; }
        return *fserving;
    }());

    if (!generator) { return generator; }

    const auto &resource(generator->resource());

    // check generator type
    if (fileInfo.generatorType != resource.generator.type) {
        return {};
    }

    return generator;
}

const Generators::Config& Generators::config() const {
    return detail().config();
}

std::vector<std::string>
Generators::Detail::listGroups(const std::string &referenceFrame
                               , Resource::Generator::Type type)
    const
{
    checkReady();

    std::vector<std::string> out;
    {
        std::unique_lock<std::mutex> lock(lock_);

        auto &idx(serving_.get<TypeIdx>());
        std::string prev;
        for (auto range(idx.equal_range(TypeKey(referenceFrame, type)));
             range.first != range.second; ++range.first)
        {
            const auto &group((*range.first)->group());
            if (group != prev) {
                out.push_back(group);
                prev = group;
            }
        }
    }

    return out;
}

std::vector<std::string>
Generators::Detail::listIds(const std::string &referenceFrame
                            , Resource::Generator::Type type
                            , const std::string &group)
    const
{
    checkReady();

    std::vector<std::string> out;
    {
        std::unique_lock<std::mutex> lock(lock_);
        auto &idx(serving_.get<GroupIdx>());
        for (auto range(idx.equal_range
                        (GroupKey(referenceFrame, type, group)));
             range.first != range.second; ++range.first)
        {
            out.push_back((*range.first)->id().id);
        }
    }

    return out;
}

std::vector<std::string>
Generators::listGroups(const std::string &referenceFrame
                       , Resource::Generator::Type type) const
{
    return detail().listGroups(referenceFrame, type);
}

std::vector<std::string>
Generators::listIds(const std::string &referenceFrame
                    , Resource::Generator::Type type
                    , const std::string &group) const
{
    return detail().listIds(referenceFrame, type, group);
}
