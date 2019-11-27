/**
 * Copyright (c) 2017 Melown Technologies SE
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

#include <thread>
#include <condition_variable>
#include <sstream>
#include <deque>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/path.hpp"
#include "utility/gccversion.hpp"
#include "utility/time.hpp"
#include "utility/raise.hpp"

#include "../error.hpp"
#include "../definition.hpp"
#include "generators.hpp"
#include "factory.hpp"

namespace fs = boost::filesystem;
namespace ba = boost::algorithm;

namespace {

Generators::Config fixUp(Generators::Config config)
{
    // sanity check
    if (!config.externalUrl.empty() && (config.externalUrl.back() != '/')) {
        config.externalUrl.push_back('/');
    }
    return config;
}

} // namespace

Generators::Detail::Detail(const Generators::Config &config
                           , const ResourceBackend::pointer &resourceBackend)
    : config_(fixUp(config)), resourceBackend_(resourceBackend)
    , arsenal_(), running_(false), updateRequest_(false), lastUpdate_(0)
    , ready_(false), preparing_(0)
    , work_(ios_), demRegistry_(std::make_shared<DemRegistry>())
{
    registerSystemGenerators();
}

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

    // invalidate any update request
    updateRequest_ = false;
    // never update
    lastUpdate_ = 0;

    while (running_) {
        // default sleep time in seconds
        std::chrono::seconds sleep(config_.resourceUpdatePeriod);

        try {
            update(resourceBackend_->load());
            lastUpdate_ = utility::usecFromEpoch();
        } catch (Aborted) {
            // pass
        } catch (const std::exception &e) {
            LOG(err2) << "Resource info update failed: <" << e.what() << ">.";
            if (config_.resourceUpdatePeriod > 0) {
                sleep = std::chrono::seconds(5);
            }
        }

        // sleep for configured time minutes
        {
            std::unique_lock<std::mutex> lock(updaterLock_);

            // condition variable wait predicate
            const auto predicate([this]() -> bool
            {
                auto updateRequest
                    (std::atomic_exchange(&updateRequest_, false));
                return !running_ || updateRequest;
            });

            if (config_.resourceUpdatePeriod > 0) {
                // wait for given duration
                updaterCond_.wait_for(lock, sleep, predicate);
            } else {
                // wait untill bugged
                updaterCond_.wait(lock, predicate);
            }
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
    ++preparing_;

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
        --preparing_;
    });
}

void Generators::Detail::registerSystemGenerators()
{
    for (const auto &ritem : Generator::Factory::registry()) {
        const auto &resourceGenerator(ritem.first);
        const auto &factory(ritem.second);

        if (!factory->systemInstance()) { continue; }

        for (const auto &rfitem : vr::system.referenceFrames) {
            const auto &rfId(rfitem.first);
            const auto &rf(rfitem.second);
            LOG(info2) << "About to register " << resourceGenerator
                       << " generator for"
                       << " reference frame " << rfitem.first << ".";

            // create resource
            Resource resource
                (resourceBackend_->genericConfig().fileClassSettings);
            resource.id = Resource::Id
                (rfId, Generator::systemGroup(), resourceGenerator.driver);
            resource.generator = resourceGenerator;
            resource.comment = "autoregistered resource";
            resource.referenceFrame = &rf;
            resource.definition(resource::definition(resourceGenerator));

            // start at first valid lod
            // lod 22 is arbitrarily deep
            resource.lodRange
                = vts::LodRange(rf.division.rootLodRange.min, 22);

            // compose lod range from first valid lode
            resource.tileRange = vts::TileRange(math::InvalidExtents{});
            for (const auto &item : rf.division.nodes) {
                const auto &node(item.second);
                if (!node.real()) { continue; }
                if (node.id.lod == rf.division.rootLodRange.min) {
                    math::update(resource.tileRange, node.id.x, node.id.y);
                }
            }

            // create generator params
            Generator::Params params(resource);
            params.config = config_;
            params.config.root = config_.root;
            params.generatorFinder = this;
            params.demRegistry = demRegistry_;
            params.system = true;

            // create generator
            auto g(factory->create(params));

            // register
            serving_.insert(g);

            // and prepare if not ready
            if (!g->ready()) {
                prepare(g);
            }
        }
    }
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
            if (!(*iserving)->system()) {
                toRemove.push_back(*iserving);
            }
            ++iserving;
        } else {
            // existing resource
            switch ((*iserving)->changed(resource)) {
            case Changed::no:
                // same stuff, do nothing
                break;

            case Changed::yes:
                // changed
                if (!config().freezes(resource.generator.type)) {
                    replace(resource, *iserving);
                }
                break;

            case Changed::safely:
            case Changed::withRevisionBump:
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
        if (!(*iserving)->system()) {
            toRemove.push_back(*iserving);
        }
    }

    // remove stuff
    for (const auto &generator : toRemove) {
        {
            std::unique_lock<std::mutex> lock(lock_);
            serving_.erase(generator);
        }
    }

    // add generators; not prepared yet to make them available to the others
    // when prepared
    for (const auto &generator : toAdd) {
        std::unique_lock<std::mutex> lock(lock_);
        serving_.insert(generator);
    }

    // prepare generators if not ready
    for (const auto &generator : toAdd) {
        if (!generator->ready()) { prepare(generator); }
    }

    // replace stuff (prepare)
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

    // wait till all pending resources are available; not nice but should work
    while (preparing_ && running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
                              , const Resource::Id &resourceId
                              , bool noReadyCheck)
    const
{
    if (!noReadyCheck) { checkReady(); }

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
                                       , const Resource::Id &resourceId
                                       , bool mustBeReady) const
{
    auto g(generator(generatorType, resourceId, !mustBeReady));
    if (!g) { return {}; }
    if (mustBeReady && !g->ready()) { return {}; }
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
        for (auto range(idx.equal_range(Keys::TypeKey(referenceFrame, type)));
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
                        (Keys::GroupKey(referenceFrame, type, group)));
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

std::uint64_t Generators::Detail::update()
{
    const auto start(utility::usecFromEpoch());
    updateRequest_ = true;
    updaterCond_.notify_one();
    return start;
}

std::uint64_t Generators::update()
{
    return detail().update();
}

bool Generators::Detail::updatedSince(std::uint64_t timestamp) const
{
    return lastUpdate_ > timestamp;
}

bool Generators::updatedSince(std::uint64_t timestamp) const
{
    return detail().updatedSince(timestamp);
}

bool Generators::Detail::has(const Resource::Id &resourceId) const
{
    std::unique_lock<std::mutex> lock(lock_);
    auto &idx(serving_.get<ResourceIdIdx>());
    auto fserving(idx.find(resourceId));
    return (fserving != idx.end());
}

bool Generators::has(const Resource::Id &resourceId) const
{
    return detail().has(resourceId);
}

bool Generators::Detail::isReady(const Resource::Id &resourceId) const
{
    std::unique_lock<std::mutex> lock(lock_);
    auto &idx(serving_.get<ResourceIdIdx>());
    auto fserving(idx.find(resourceId));
    if (fserving == idx.end()) { return false; }
    return (*fserving)->ready();
}

bool Generators::isReady(const Resource::Id &resourceId) const
{
    return detail().isReady(resourceId);
}

std::string Generators::Detail::url(const Resource::Id &resourceId
                                    , GeneratorInterface::Interface iface)
    const
{
    std::unique_lock<std::mutex> lock(lock_);
    auto &idx(serving_.get<ResourceIdIdx>());
    auto fserving(idx.find(resourceId));
    if (fserving == idx.end()) {
        LOGTHROW(err1, UnknownGenerator)
            << "No such generator <" << resourceId << ">";
    }
    return (*fserving)->url(iface);
}

std::string Generators::url(const Resource::Id &resourceId
                            , GeneratorInterface::Interface iface) const
{
    return detail().url(resourceId, iface);
}

bool Generators::Detail::updatedSince(const Resource::Id &resourceId
                                      , std::uint64_t timestamp
                                      , bool nothrow) const
{
    std::unique_lock<std::mutex> lock(lock_);
    auto &idx(serving_.get<ResourceIdIdx>());
    auto fserving(idx.find(resourceId));
    if (fserving == idx.end()) {
        if (nothrow) { return false; }
        LOGTHROW(err1, UnknownGenerator)
            << "No such generator <" << resourceId << ">";
    }
    return (*fserving)->updatedSince(timestamp);
}

bool Generators::updatedSince(const Resource::Id &resourceId
                              , std::uint64_t timestamp, bool nothrow) const
{
    return detail().updatedSince(resourceId, timestamp, nothrow);
}

void Generator::status(std::ostream &os) const
{
    os << "<" << id()
       << "> (type <" << resource().generator << ">)"
       << (ready_ ? "" : " not ready")
       << "\n";
}

void Generators::Detail::listResources(std::ostream &os) const
{

    Generator::list generators;
    {
        std::unique_lock<std::mutex> lock(lock_);
        for (const auto &generator : serving_) {
            generators.push_back(generator);
        }
    }

    for (const auto &generator : generators) {
        generator->status(os);
    }
}

void Generators::listResources(std::ostream &os) const
{
    detail().listResources(os);
}
