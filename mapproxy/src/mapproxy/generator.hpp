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

#ifndef mapproxy_generator_hpp_included_
#define mapproxy_generator_hpp_included_

#include <memory>
#include <string>
#include <map>
#include <iostream>
#include <atomic>
#include <cstdint>

#include <boost/noncopyable.hpp>
#include <boost/any.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/resourcefetcher.hpp"
#include "utility/format.hpp"

#include "vts-libs/storage/support.hpp"
#include "vts-libs/vts/mapconfig.hpp"

#include "resource.hpp"
#include "resourcebackend.hpp"
#include "fileinfo.hpp"
#include "gdalsupport.hpp"
#include "sink.hpp"

#include "generator/demregistry.hpp"

namespace vs = vtslibs::storage;
namespace vts = vtslibs::vts;

struct Arsenal {
    GdalWarper &warper;
    const utility::ResourceFetcher &fetcher;

    Arsenal(GdalWarper &warper, const utility::ResourceFetcher &fetcher)
        : warper(warper), fetcher(fetcher)
    {}
};

class Generator;

class GeneratorFinder {
public:
    virtual ~GeneratorFinder() {}

    std::shared_ptr<Generator>
    findGenerator(Resource::Generator::Type generatorType
                  , const Resource::Id &resourceId
                  , bool mustBeReady = true) const;

private:
    virtual std::shared_ptr<Generator>
    findGenerator_impl(Resource::Generator::Type generatorType
                       , const Resource::Id &resourceId
                       , bool mustBeReady) const = 0;
};

struct GeneratorNotFound : std::runtime_error {
    GeneratorNotFound(Resource::Generator::Type generatorType
                      , const Resource::Id &resourceId)
        : std::runtime_error
          (utility::format("Cannot find <%s> generator for resource <%s>."
                           , generatorType, resourceId))
        , generatorType(generatorType)
        , resourceId(resourceId)
    {}

    Resource::Generator::Type generatorType;
    const Resource::Id &resourceId;
};

/** Dataset generator.
 */
class Generator : boost::noncopyable {
public:
    typedef std::shared_ptr<Generator> pointer;
    typedef std::vector<pointer> list;
    typedef std::map<Resource::Id, pointer> map;
    typedef std::function<void(Sink&, Arsenal&)> Task;

    /** Configuration
     */
    struct Config {
        boost::filesystem::path root;
        boost::filesystem::path resourceRoot;
        boost::filesystem::path tmpRoot;
        int fileFlags;
        const vs::SupportFile::Vars *variables;
        const vs::SupportFile::Vars *defaults;
        double defaultFov;
        std::set<Resource::Generator::Type> freezeResourceTypes;
        std::string externalUrl;

        Config()
            : fileFlags(), variables(), defaults()
            , defaultFov(vr::Position::naturalFov())
            , freezeResourceTypes{Resource::Generator::Type::surface}
        {}

        bool freezes(Resource::Generator::Type type) const {
            return freezeResourceTypes.find(type) != freezeResourceTypes.end();
        }
    };

    virtual ~Generator() {}

    struct Params {
        Config config;
        Resource resource;
        const GeneratorFinder *generatorFinder;
        DemRegistry::pointer demRegistry;
        Generator::pointer replace;
        bool system;

        Params(const Resource &resource)
            : resource(resource), system(false) {}
    };

    struct Properties {
        GeneratorInterface::Interface supportedInterfaces;

        Properties(GeneratorInterface::Interface iface
                       = GeneratorInterface::Interface::vts)
            : supportedInterfaces(iface)
        {}

        Properties& support(GeneratorInterface::Interface iface) {
            supportedInterfaces |= iface; return *this;
        }

        bool isSupported(GeneratorInterface::Interface iface) const {
            return (supportedInterfaces & iface) == iface;
        }
    };

    static Generator::pointer create(const Params &params);

    static DefinitionBase::pointer definition(const Resource::Generator &type);

    struct Factory;
    static void registerType(const Resource::Generator &type
                             , const std::shared_ptr<Factory> &factory);

    template <typename GeneratorType>
    static void registerType(const std::shared_ptr<Factory> &factory) {
        registerType({ GeneratorType::Definition::type
                       , GeneratorType::Definition::driverName }
                     , factory);
    }

    /** Generator is ready when all data needed to serve are prepared.
     */
    bool ready() const { return ready_; }

    /** Throws Unavailable if generator is not ready yet.
     */
    void checkReady() const;

    bool system() const { return system_; }

    /** Prepares generator for serving.
     */
    void prepare(Arsenal &arsenal);

    const Resource& resource() const { return resource_; }
    const Resource::Id& id() const { return resource_.id; }
    const std::string& group() const { return resource_.id.group; }
    Resource::Generator::Type type() const { return resource_.generator.type; }
    const std::string& referenceFrameId() const {
        return resource_.id.referenceFrame;
    }
    const vr::ReferenceFrame& referenceFrame() const {
        return *resource_.referenceFrame;
    }

    const Config& config() const { return config_; }
    const boost::filesystem::path& root() const { return config_.root; }
    const boost::filesystem::path& resourceRoot() const {
        return config_.resourceRoot;
    }
    const Properties& properties() const { return properties_; }

    Changed changed(const Resource &resource) const;

    vts::MapConfig mapConfig(ResourceRoot root) const;

    Task generateFile(const FileInfo &fileInfo, Sink sink) const;

    void status(std::ostream &os) const;

    /** Pointer to original generator this one replaces.
     *  Used in runtime update.
     */
    Generator::pointer replace() const { return replace_; }

    static std::string systemGroup() { return ".system"; }

    /** Writes any pending changes.
     */
    void commitEnforcedChange();

    /** Build resource URL under config.externalUrl.
     */
    std::string url(GeneratorInterface::Interface iface
                    = GeneratorInterface::Interface::vts)
        const;

    /** Was the resource updated since given timestamp.
     */
    bool updatedSince(std::uint64_t timestamp) const;

    /** Generic type for provider handling
     */
    struct Provider { virtual ~Provider() {} };

    /** Returns generators provider machinery. Returns null if provider of given
     *  type is not available.
     */
    template <typename ProviderType>
    ProviderType* getProvider() const;

protected:
    Generator(const Params &params, const Properties &props = Properties());

    void makeReady();
    bool fresh() const { return fresh_; }

    void mapConfig(std::ostream &os, ResourceRoot root) const;

    std::string absoluteDataset(const std::string &path) const;
    boost::filesystem::path
    absoluteDataset(const boost::filesystem::path &path) const;

    boost::optional<std::string>
    absoluteDataset(const boost::optional<std::string> &path) const;

    std::string absoluteDataset(const std::string &path
                                , const boost::optional<std::string> &override)
        const;

    /** Same as absoluteDataset but adds reference frame as an extension to the
     *  filename.
     */
    boost::optional<boost::filesystem::path>
    absoluteDatasetRf(const boost::optional<boost::filesystem::path> &path)
        const;

    /** Returns valid path if given path is valid and reference file exists.
     */
    boost::optional<boost::filesystem::path>
    ignoreNonexistent(const boost::optional<boost::filesystem::path> &path)
        const;

    Generator::pointer otherGenerator(Resource::Generator::Type generatorType
                                      , const Resource::Id &resourceId
                                      , bool mustBeReady = true
                                      , bool mandatory = false) const;

    void supportFile(const vs::SupportFile &support, Sink &sink
                     , const Sink::FileInfo &fileInfo) const;

    DemRegistry& demRegistry() { return *demRegistry_; }
    const DemRegistry& demRegistry() const { return *demRegistry_; }

    /** This function must be checked in derived class ctor and resource must
     *  not be made ready.
     */
    bool changeEnforced() const { return changeEnforced_; }

    /** Sets new provider. Value is stolen.
     */
    void setProvider(std::unique_ptr<Provider> &&provider);

private:
    virtual void prepare_impl(Arsenal &arsenal) = 0;
    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const = 0;

    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , Sink &sink) const = 0;

    const GeneratorFinder *generatorFinder_;
    Config config_;
    Properties properties_;
    Resource resource_;
    Resource savedResource_;
    bool fresh_;
    bool system_;
    bool changeEnforced_;
    std::atomic<bool> ready_;
    std::atomic<std::uint64_t> readySince_;
    DemRegistry::pointer demRegistry_;
    Generator::pointer replace_;
    std::unique_ptr<Provider> provider_;
};

/** Set of dataset generators.
 */
class Generators : public boost::noncopyable {
public:
    struct Config : Generator::Config {
        int resourceUpdatePeriod;

        Config() : resourceUpdatePeriod(100) {}
    };

    /** Creates generator set.
     */
    Generators(const Config &config
               , const ResourceBackend::pointer &resourceBackend);

    ~Generators();

    void start(Arsenal &arsenal);
    void stop();

    const Config& config() const;

    /** Returns generator for requested file.
     */
    Generator::pointer generator(const FileInfo &fileInfo) const;

    /** Returns generator for requested type and resourceId.
     */
    Generator::pointer generator(Resource::Generator::Type generatorType
                                 , const Resource::Id &resourceId) const;

    /** Returns list of all generators for given referenceFrame.
     */
    Generator::list referenceFrame(const std::string &referenceFrame) const;

    std::vector<std::string> listGroups(const std::string &referenceFrame
                                        , Resource::Generator::Type type)
        const;

    std::vector<std::string> listIds(const std::string &referenceFrame
                                     , Resource::Generator::Type type
                                     , const std::string &group) const;

    const DemRegistry& demRegistry() const;

    /** Force resources update.
     */
    std::uint64_t update();

    bool updatedSince(std::uint64_t timestamp) const;

    bool updatedSince(const Resource::Id &resourceId
                      , std::uint64_t timestamp, bool nothrow = false)
        const;

    void listResources(std::ostream &os) const;

    bool has(const Resource::Id &resourceId) const;

    bool isReady(const Resource::Id &resourceId) const;

    std::string url(const Resource::Id &resourceId
                    , GeneratorInterface::Interface
                    = GeneratorInterface::Interface::vts) const;

    // internals
    struct Detail;

private:
    std::shared_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};

// inlines

inline void Generator::prepare(Arsenal &arsenal)
{
    // prepare only when not ready
    if (ready_) { return; }

    // prepare and make ready
    prepare_impl(arsenal);
    makeReady();
}

inline vts::MapConfig Generator::mapConfig(ResourceRoot root) const
{
    return mapConfig_impl(root);
}

inline Generator::pointer Generators::generator(const FileInfo &fileInfo) const
{
    return generator(fileInfo.interface, fileInfo.resourceId);
}

std::shared_ptr<Generator>
inline GeneratorFinder::findGenerator(Resource::Generator::Type generatorType
                                      , const Resource::Id &resourceId
                                      , bool mustBeReady) const
{
    return findGenerator_impl(generatorType, resourceId, mustBeReady);
}

inline Generator::pointer
Generator::otherGenerator(Resource::Generator::Type generatorType
                          , const Resource::Id &resourceId
                          , bool mustBeReady, bool mandatory)
    const
{
    auto other(generatorFinder_->findGenerator
               (generatorType, resourceId, mustBeReady));
    if (!other && mandatory) {
        throw GeneratorNotFound(generatorType, resourceId);
    }
    return other;
}

template <typename ProviderType>
ProviderType* Generator::getProvider() const
{
    if (!provider_) { return nullptr; }
    return dynamic_cast<ProviderType*>(provider_.get());
}

inline void Generator::setProvider(std::unique_ptr<Provider> &&provider)
{
    provider_ = std::move(provider);
}

#endif // mapproxy_generator_hpp_included_
