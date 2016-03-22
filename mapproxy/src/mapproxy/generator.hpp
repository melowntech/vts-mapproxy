#ifndef mapproxy_generator_hpp_included_
#define mapproxy_generator_hpp_included_

#include <memory>
#include <string>
#include <iostream>
#include <atomic>

#include <boost/noncopyable.hpp>
#include <boost/any.hpp>
#include <boost/filesystem/path.hpp>

#include "./resources.hpp"
#include "./resourcebackend.hpp"

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

protected:
    Generator(const boost::filesystem::path &root
              , const Resource &resource)
        : root_(root), resource_(resource), ready_(false)
    {}

    virtual void prepare_impl() = 0;

    const boost::filesystem::path root_;
    const Resource resource_;
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

    /** Matches URL path and returns appropriate generator or null in case of no
     *  match.
     */
    Generator::pointer matchUrl(const std::string &path);

    /** Returns list of all generators for given referenceFrame.
     */
    Generator::list referenceFrame(const std::string &referenceFrame);

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

#endif // mapproxy_generator_hpp_included_
