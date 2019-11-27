/**
 * Copyright (c) 2019 Melown Technologies SE
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

#ifndef mapproxy_generator_generators_hpp_included_
#define mapproxy_generator_generators_hpp_included_

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/global_fun.hpp>
#include <boost/multi_index/mem_fun.hpp>

#include "../generator.hpp"

namespace asio = boost::asio;
namespace bmi = boost::multi_index;

namespace Keys {

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

    static TypeKey extract(const Generator &generator) {
        const auto r(generator.resource());
        return { r.id.referenceFrame, r.generator.type };
    }
};

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

    static GroupKey extract(const Generator &generator) {
        const auto r(generator.resource());
        return { r.id.referenceFrame, r.generator.type, r.id.group };
    }

};

} // namespace Keys

class Generators::Detail
    : public boost::noncopyable
    , public GeneratorFinder
{
public:
    Detail(const Generators::Config &config
           , const ResourceBackend::pointer &resourceBackend);

    void checkReady() const;

    Generator::pointer generator(Resource::Generator::Type generatorType
                                 , const Resource::Id &resourceId
                                 , bool noReadyCheck = false) const;

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

    std::uint64_t update();

    bool updatedSince(std::uint64_t timestamp) const;

    void listResources(std::ostream &os) const;

    void replace(const Generator::pointer &original
                 , const Generator::pointer &replacement);

    bool has(const Resource::Id &resourceId) const;

    bool isReady(const Resource::Id &resourceId) const;

    std::string url(const Resource::Id &resourceId
                    , GeneratorInterface::Interface iface) const;

    bool updatedSince(const Resource::Id &resourceId
                      , std::uint64_t timestamp, bool nothrow) const;

private:
    void registerSystemGenerators();

    void update(const Resource::map &resources);

    void updater();
    void worker(std::size_t id);
    void prepare(const Generator::pointer &generator);

    virtual Generator::pointer
    findGenerator_impl(Resource::Generator::Type generatorType
                       , const Resource::Id &resourceId
                       , bool mustBeReady) const;

    const Config config_;
    ResourceBackend::pointer resourceBackend_;
    Arsenal *arsenal_;

    // resource updater stuff
    std::thread updater_;
    std::atomic<bool> running_;
    std::atomic<bool> updateRequest_;
    std::atomic<std::uint64_t> lastUpdate_;
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
                    , bmi::global_fun<const Generator&, Keys::TypeKey
                                      , &Keys::TypeKey::extract>
                    >

              , bmi::ordered_non_unique<
                    bmi::tag<GroupIdx>
                    , bmi::global_fun<const Generator&, Keys::GroupKey
                                      , &Keys::GroupKey::extract>
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
    std::atomic<int> preparing_;

    // prepare stuff
    asio::io_service ios_;
    asio::io_service::work work_;
    std::vector<std::thread> workers_;

    // DEM registry
    DemRegistry::pointer demRegistry_;
};

#endif // mapproxy_generator_generators_hpp_included_
