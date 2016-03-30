#ifndef mapproxy_generator_hpp_included_
#define mapproxy_generator_hpp_included_

#include <memory>
#include <string>
#include <iostream>
#include <atomic>

#include <boost/noncopyable.hpp>
#include <boost/any.hpp>
#include <boost/filesystem/path.hpp>

#include "vts-libs/vts/mapconfig.hpp"

#include "./resource.hpp"
#include "./resourcebackend.hpp"
#include "./fileinfo.hpp"
#include "./contentgenerator.hpp"

namespace vts = vadstena::vts;

/** Dataset generator.
 */
class Generator : boost::noncopyable {
public:
    typedef std::shared_ptr<Generator> pointer;
    typedef std::vector<pointer> list;
    typedef std::map<Resource::Id, pointer> map;
    typedef std::function<void()> Task;

    /** Configuration
     */
    struct Config {
        boost::filesystem::path root;
        int fileFlags;

        Config() : fileFlags() {}
    };

    virtual ~Generator() {}

    static Generator::pointer create(const Config &config
                                     , const Resource::Generator &type
                                     , const Resource &resource);

    struct Factory;
    static void registerType(const Resource::Generator &type
                             , const std::shared_ptr<Factory> &factory);

    /** Generator is ready when all data needed to serve are prepared.
     */
    bool ready() const { return ready_; }

    /** Prepares generator for serving.
     */
    void prepare();

    const Resource& resource() const { return resource_; }
    const Resource::Id& id() const { return resource_.id; }
    const Config& config() const { return config_; }
    const boost::filesystem::path& root() const { return config_.root; }

    bool check(const Resource &resource) const;

    bool handlesReferenceFrame(const std::string &referenceFrame) const;

    vts::MapConfig mapConfig(const std::string &referenceFrame
                             , ResourceRoot root) const;

    Task generateFile(const FileInfo &fileInfo
                      , const Sink::pointer &sink) const;

protected:
    Generator(const Config &config, const Resource &resource);

    void makeReady();
    bool fresh() const { return fresh_; }

    void mapConfig(std::ostream &os, const std::string &referenceFrame
                   , ResourceRoot root) const;

private:
    virtual void prepare_impl() = 0;
    virtual vts::MapConfig
    mapConfig_impl(const std::string &referenceFrame
                   , ResourceRoot root) const = 0;

    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , const Sink::pointer &sink) const = 0;

    const Config config_;
    Resource resource_;
    Resource savedResource_;
    bool fresh_;
    std::atomic<bool> ready_;
};

/** Set of dataset generators.
 */
class Generators {
public:
    struct Config {
        boost::filesystem::path root;
        int fileFlags;
        int resourceUpdatePeriod;

        Config() : fileFlags(), resourceUpdatePeriod(100) {}
    };

    /** Creates generator set.
     */
    Generators(const Config &config
               , const ResourceBackend::pointer &resourceBackend);

    ~Generators();

    /** Returns generator for requested file.
     */
    Generator::pointer generator(const FileInfo &fileInfo) const;

    /** Returns list of all generators for given referenceFrame.
     */
    Generator::list referenceFrame(const std::string &referenceFrame) const;

    // internals
    struct Detail;

private:
    std::shared_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};

// inlines

inline void Generator::prepare()
{
    prepare_impl();
}

inline vts::MapConfig
Generator::mapConfig(const std::string &referenceFrame
                     , ResourceRoot root) const
{
    return mapConfig_impl(referenceFrame, root);
}

inline Generator::Task Generator::generateFile(const FileInfo &fileInfo
                                               , const Sink::pointer &sink)
    const
{
    return generateFile_impl(fileInfo, sink);
}

inline bool Generator::handlesReferenceFrame(const std::string &referenceFrame)
    const
{
    return resource_.referenceFrames.count(referenceFrame);
}

#endif // mapproxy_generator_hpp_included_
