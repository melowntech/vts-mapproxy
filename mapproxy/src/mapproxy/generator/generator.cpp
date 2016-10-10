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

Generator::pointer Generator::create(const Params &params)
{
    try {
        return findFactory(params.resource.generator)->create(params);
    } catch (const boost::bad_any_cast&) {
        LOGTHROW(err2, InvalidConfiguration)
            << "Passed resource does not match generator <"
            << params.resource.generator << ">.";
    }
    throw;
}

Generator::Generator(const Params &params)
    : generatorFinder_(params.generatorFinder), config_(params.config)
    , resource_(params.resource), savedResource_(params.resource)
    , fresh_(false), ready_(false)
    , demRegistry_(params.demRegistry)
    , replace_(params.replace)
{
    config_.root = (config_.root / resource_.id.referenceFrame
                    / resource_.id.group / resource_.id.id);

    // TODO: handle failed creation
    auto rfile(root() / ResourceFile);

    if (create_directories(root())) {
        // new resource
        fresh_ = true;
        save(rfile, resource_);
    } else {
        // reopen of existing dataset
        savedResource_
            = loadResource(rfile).front();
        switch (savedResource_.changed(resource_)) {
        case Changed::no:
        case Changed::safely:
            // nothing or something non-destructive changed -> re-save
            save(rfile, resource_);
            break;

        case Changed::yes:
            // different setup, use stored definition
            LOG(warn3)
                << "Definition of resource <" << resource_.id
                << "> differs from the one stored in store at "
                << root() << "; using stored definition.";
            resource_ = savedResource_;

            // force received file class setings even for saved resource
            resource_.fileClassSettings = params.resource.fileClassSettings;
            break;
        }
    }
}

Changed Generator::changed(const Resource &resource) const
{
    switch (auto changed = resource.changed(resource_)) {
    case Changed::yes:
        LOG(warn2)
            << "Definition of resource <" << resource.id
            << "> differs from the one stored in store at "
            << root() << "; using stored definition.";
        return changed;

    default:
        return changed;
    }
}

void Generator::makeReady()
{
    ready_ = true;
    LOG(info2) << "Ready to serve resource <" << id()
               << "> (type <" << resource().generator << ">).";
}

void Generator::mapConfig(std::ostream &os, ResourceRoot root)
    const
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

class Generators::Detail
    : public boost::noncopyable
    , public GeneratorFinder
{
public:
    Detail(const Generators::Config &config
           , const ResourceBackend::pointer &resourceBackend)
        : config_(config), resourceBackend_(resourceBackend)
        , arsenal_(), running_(false), ready_(false), work_(ios_)
        , demRegistry_(std::make_shared<DemRegistry>())
    {}

    void checkReady() const;

    Generator::pointer generator(Resource::Generator::Type generatorType
                                 , const Resource::Id &resourceId) const;

    Generator::list referenceFrame(const std::string &referenceFrame) const;

    std::vector<std::string> listGroups(const std::string &referenceFrame
                                        , Resource::Generator::Type type)
        const;

    std::vector<std::string> listIds(const std::string &referenceFrame
                                     , Resource::Generator::Type type
                                     , const std::string &group) const;

    void start(Arsenal &arsenal);
    void stop();

    inline const Config& config() const { return config_; }

    inline const DemRegistry& demRegistry() const { return *demRegistry_; }

    void update();

    void stat(std::ostream &os) const;

    void replace(const Generator::pointer &original
                 , const Generator::pointer &replacement);

private:
    void update(const Resource::map &resources);

    void updater();
    void worker(std::size_t id);
    void prepare(const Generator::pointer &generator);

    virtual Generator::pointer
    findGenerator_impl(Resource::Generator::Type generatorType
                       , const Resource::Id &resourceId) const;

    const Config config_;
    ResourceBackend::pointer resourceBackend_;
    Arsenal *arsenal_;

    // resource updater stuff
    std::thread updater_;
    std::atomic<bool> running_;
    std::atomic<bool> updateRequest_;
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

    // DEM registry
    DemRegistry::pointer demRegistry_;
};

void Generators::Detail::checkReady() const
{
    if (ready_) { return; }
    throw Unavailable("Server not ready.");
}

void Generators::Detail::start(Arsenal &arsenal)
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

    arsenal_ = &arsenal;
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
    arsenal_ = {};
}

struct Aborted {};

void Generators::Detail::updater()
{
    dbglog::thread_id("updater");

    updateRequest_ = false;

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
            updaterCond_.wait_for(lock, sleep, [this]() -> bool
            {
                auto updateRequest
                    (std::atomic_exchange(&updateRequest_, false));
                return !running_ || updateRequest;
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
            generator->prepare(*arsenal_);
            if (auto original = generator->replace()) {
                replace(original, generator);
            }
        } catch (const std::exception &e) {
            LOG(warn2)
                << "Failed to prepare generator for <"
                << generator->resource().id << "> (" << e.what()
                << "); removing from set of known generators.";

            resourceBackend_->error(generator->resource().id, e.what());

            // erease from map (under lock)
            std::unique_lock<std::mutex> lock(lock_);
            serving_.erase(generator);
        }
    });
}

Generators::Generators(const Config &config
                       , const ResourceBackend::pointer &resourceBackend)
    : detail_(std::make_shared<Detail>(config, resourceBackend))
{}

Generators::~Generators()
{
    detail().stop();
}

void Generators::start(Arsenal &arsenal)
{
    detail().start(arsenal);
}

void Generators::stop()
{
    detail().stop();
}

Generator::list Generators::referenceFrame(const std::string &referenceFrame)
    const
{
    return detail().referenceFrame(referenceFrame);
}

Generator::pointer
Generators::generator(Resource::Generator::Type generatorType
                      , const Resource::Id &resourceId) const
{
    return detail().generator(generatorType, resourceId);
}

void Generators::Detail::replace(const Generator::pointer &original
                                 , const Generator::pointer &replacement)
{
    std::unique_lock<std::mutex> lock(lock_);
    // find original in the serving set
    auto ioriginal(serving_.find(original));
    // and replace
    serving_.replace(ioriginal, replacement);
    LOG(info3)
        << "Replaced resource <" << original->id() << "> with new definiton.";
}

void Generators::Detail::update(const Resource::map &resources)
{
    LOG(info2) << "Updating resources.";

    auto iresources(resources.begin()), eresources(resources.end());
    auto &idx(serving_.get<ResourceIdIdx>());
    auto iserving(idx.begin()), eserving(idx.end());

    Generator::list toAdd;
    Generator::list toRemove;
    Generator::list toReplace;

    auto add([&](const Resource &res)
    {
        if (!running_) {
            throw Aborted{};
        }
        try {
            Generator::Params params(res);
            params.config = config_;
            params.config.root = config_.root;
            params.generatorFinder = this;
            params.demRegistry = demRegistry_;
            toAdd.push_back(Generator::create(params));
        } catch (const std::exception &e) {
            LOG(err2) << "Failed to create generator for resource <"
                      << iresources->first << ">: <" << e.what() << ">.";
        }
    });

    auto replace([&](const Resource &res, const Generator::pointer &original)
    {
        if (!running_) {
            throw Aborted{};
        }
        try {
            Generator::Params params(res);
            params.config = config_;
            params.config.root = config_.root;
            params.generatorFinder = this;
            params.demRegistry = demRegistry_;
            params.replace = original;
            toReplace.push_back(Generator::create(params));
        } catch (const std::exception &e) {
            LOG(err2) << "Failed to re-create generator for resource <"
                      << iresources->first << ">: <" << e.what() << ">.";
        }
    });

    // process common stuff
    while ((iresources != eresources) && (iserving != eserving)) {
        const auto &resource(iresources->second);
        if (iresources->first < (*iserving)->id()) {
            // new resource
            add(resource);
            ++iresources;
        } else if ((*iserving)->id() < iresources->first) {
            // removed resource
            toRemove.push_back(*iserving);
            ++iserving;
        } else {
            // existing resource
            switch ((*iserving)->changed(resource)) {
            case Changed::no:
            case Changed::yes:
                // same or completety different stuff, do nothing
                break;

            case Changed::safely:
                // here comes the fun
                replace(resource, *iserving);
                break;
            }

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

    // repalce stuff (prepare)
    for (const auto &generator : toReplace) {
        if (!generator->ready()) {
            prepare(generator);
        } else {
            this->replace(generator->replace(), generator);
        }
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

Generator::pointer
Generators::Detail::generator(Resource::Generator::Type generatorType
                              , const Resource::Id &resourceId)
    const
{
    checkReady();

    // find generator (under lock)
    auto generator([&]() -> Generator::pointer
    {
        std::unique_lock<std::mutex> lock(lock_);
        auto &idx(serving_.get<ResourceIdIdx>());
        auto fserving(idx.find(resourceId));
        if (fserving == idx.end()) { return {}; }
        return *fserving;
    }());

    if (!generator) { return generator; }

    const auto &resource(generator->resource());

    // check generator type
    if (generatorType != resource.generator.type) {
        return {};
    }

    return generator;
}

Generator::pointer
Generators::Detail::findGenerator_impl(Resource::Generator::Type generatorType
                                       , const Resource::Id &resourceId) const
{
    auto g(generator(generatorType, resourceId));
    if (!g || !g->ready()) { return {}; }
    return g;
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

void Generator::supportFile(const vs::SupportFile &support, Sink &sink
                            , const Sink::FileInfo &fileInfo) const
{
    if (!support.isTemplate) {
        sink.content(support.data, support.size, fileInfo, false);
        return;
    }

    // expand and send
    sink.content(support.expand(config_.variables, config_.defaults)
                 , fileInfo);
}

const DemRegistry& Generators::demRegistry() const
{
    return detail().demRegistry();
}

void Generators::Detail::update()
{
    updateRequest_ = true;
    updaterCond_.notify_one();
}

void Generators::update()
{
    detail().update();
}

void Generator::stat(std::ostream &os) const
{
    os << "<" << id()
       << "> (type <" << resource().generator << ">)"
       << (ready_ ? "" : " not ready")
       << "\n";
}

void Generators::Detail::stat(std::ostream &os) const
{

    Generator::list generators;
    {
        std::unique_lock<std::mutex> lock(lock_);
        for (const auto &generator : serving_) {
            generators.push_back(generator);
        }
    }

    for (const auto &generator : generators) {
        generator->stat(os);
    }
}

void Generators::stat(std::ostream &os) const
{
    detail().stat(os);
}
