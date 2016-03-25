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

namespace vts = vadstena::vts;

/** Dataset generator.
 */
class Generator : boost::noncopyable {
public:
    typedef std::shared_ptr<Generator> pointer;
    typedef std::vector<pointer> list;
    typedef std::map<Resource::Id, pointer> map;

    virtual ~Generator() {}

    static Generator::pointer create(const Resource::Generator &type
                                     , const boost::filesystem::path &root
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
    const boost::filesystem::path& root() const { return root_; }

    bool check(const Resource &resource) const;

    bool handlesReferenceFrame(const std::string &referenceFrame) const;

    vts::MapConfig mapConfig(const std::string &referenceFrame
                             , const boost::filesystem::path &root) const;

protected:
    Generator(const boost::filesystem::path &root
              , const Resource &resource);

    void makeReady();
    bool fresh() const { return fresh_; }

private:
    virtual void prepare_impl() = 0;
    virtual vts::MapConfig
    mapConfig_impl(const std::string &referenceFrame
                   , const boost::filesystem::path &root) const = 0;

    const boost::filesystem::path root_;
    Resource resource_;
    Resource savedResource_;
    bool fresh_;
    std::atomic<bool> ready_;
};

/** Set of dataset generators.
 */
class Generators {
public:
    /** Creates generator set.
     */
    Generators(const boost::filesystem::path &root
               , const ResourceBackend::pointer &resourceBackend
               , int resourceUpdatePeriod);

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
                     , const boost::filesystem::path &root) const
{
    return mapConfig_impl(referenceFrame, root);
}

inline bool Generator::handlesReferenceFrame(const std::string &referenceFrame)
    const
{
    return resource_.referenceFrames.count(referenceFrame);
}

#endif // mapproxy_generator_hpp_included_
