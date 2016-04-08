#include <thread>
#include <condition_variable>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/global_fun.hpp>
#include <boost/multi_index/mem_fun.hpp>

#include "dbglog/dbglog.hpp"

#include "../error.hpp"
#include "../generator.hpp"
#include "./factory.hpp"

namespace fs = boost::filesystem;
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

boost::optional<std::string>
Generator::absoluteDataset(const boost::optional<std::string> &path) const
{
    if (!path) { return path; }
    return absoluteDataset(*path);
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
        , updaterRunning_(false)
    {}

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

    fs::path makePath(const Resource::Id &id);

    void runUpdater();

    const Config config_;
    ResourceBackend::pointer resourceBackend_;

    // resource updater stuff
    std::thread updater_;
    std::atomic<bool> updaterRunning_;
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
    mutable std::mutex servingLock_;
    GeneratorMap serving_;

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

struct Aborted {};

void Generators::Detail::runUpdater()
{
    dbglog::thread_id("updater");


    while (updaterRunning_) {
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
                return !updaterRunning_;
            });
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
    auto &idx(serving_.get<ResourceIdIdx>());
    auto iserving(idx.begin()), eserving(idx.end());

    Generator::list toAdd;
    Generator::list toRemove;

    auto add([&](const Resource &res)
    {
        if (!updaterRunning_) {
            throw Aborted{};
        }
        try {
            Generator::Config config(config_);
            config.root = makePath(res.id);
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
            std::unique_lock<std::mutex> lock(servingLock_);
            serving_.insert(generator);
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
            serving_.erase(generator);
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
    // find generator (under lock)
    auto generator([&]() -> Generator::pointer
    {
        std::unique_lock<std::mutex> lock(servingLock_);
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
    std::vector<std::string> out;
    {
        std::unique_lock<std::mutex> lock(servingLock_);

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
    std::vector<std::string> out;
    {
        std::unique_lock<std::mutex> lock(servingLock_);
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
