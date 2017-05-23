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

#include <boost/noncopyable.hpp>
#include <boost/any.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/resourcefetcher.hpp"

#include "vts-libs/storage/support.hpp"
#include "vts-libs/vts/mapconfig.hpp"

#include "./resource.hpp"
#include "./resourcebackend.hpp"
#include "./fileinfo.hpp"
#include "./gdalsupport.hpp"
#include "./sink.hpp"

#include "./generator/demregistry.hpp"

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
                  , const Resource::Id &resourceId) const;

private:
    virtual std::shared_ptr<Generator>
    findGenerator_impl(Resource::Generator::Type generatorType
                       , const Resource::Id &resourceId) const = 0;
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

        Config() : fileFlags(), variables(), defaults() {}
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

    static Generator::pointer create(const Params &params);

    static DefinitionBase::pointer definition(const Resource::Generator &type);

    struct Factory;
    static void registerType(const Resource::Generator &type
                             , const std::shared_ptr<Factory> &factory);

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

    Changed changed(const Resource &resource) const;

    vts::MapConfig mapConfig(ResourceRoot root) const;

    Task generateFile(const FileInfo &fileInfo, Sink sink) const;

    void stat(std::ostream &os) const;

    /** Pointer to original generator this one replaces.
     *  Used in runtime update.
     */
    Generator::pointer replace() const { return replace_; }

    static std::string systemGroup() { return ".system"; }

protected:
    Generator(const Params &params);

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
                                      , const Resource::Id &resourceId) const;

    void supportFile(const vs::SupportFile &support, Sink &sink
                     , const Sink::FileInfo &fileInfo) const;

    DemRegistry& demRegistry() { return *demRegistry_; }
    const DemRegistry& demRegistry() const { return *demRegistry_; }

private:
    virtual void prepare_impl(Arsenal &arsenal) = 0;
    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const = 0;

    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , Sink &sink) const = 0;

    const GeneratorFinder *generatorFinder_;
    Config config_;
    Resource resource_;
    Resource savedResource_;
    bool fresh_;
    bool system_;
    std::atomic<bool> ready_;
    DemRegistry::pointer demRegistry_;
    Generator::pointer replace_;
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
    void update();

    void stat(std::ostream &os) const;

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
    // prepare only when ready
    if (ready_) { return; }

    // prepare and make ready
    prepare_impl(arsenal);
    makeReady();
}

inline vts::MapConfig Generator::mapConfig(ResourceRoot root) const
{
    return mapConfig_impl(root);
}

inline Generator::Task Generator::generateFile(const FileInfo &fileInfo
                                               , Sink sink)
    const
{
    return generateFile_impl(fileInfo, sink);
}

inline Generator::pointer Generators::generator(const FileInfo &fileInfo) const
{
    return generator(fileInfo.generatorType, fileInfo.resourceId);
}

std::shared_ptr<Generator>
inline GeneratorFinder::findGenerator(Resource::Generator::Type generatorType
                                      , const Resource::Id &resourceId) const
{
    return findGenerator_impl(generatorType, resourceId);
}

inline Generator::pointer
Generator::otherGenerator(Resource::Generator::Type generatorType
                          , const Resource::Id &resourceId) const
{
    return generatorFinder_->findGenerator(generatorType, resourceId);
}

#endif // mapproxy_generator_hpp_included_
