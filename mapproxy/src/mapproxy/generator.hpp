#ifndef mapproxy_generator_hpp_included_
#define mapproxy_generator_hpp_included_

#include <memory>
#include <string>
#include <iostream>

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

    virtual ~Generator() {}

    static Generator::pointer create(const std::string &type
                                     , const boost::filesystem::path &root
                                     , const Resource &resource);

    struct Factory;
    static void registerType(const std::string &type
                             , const std::shared_ptr<Factory> &factory);

    /** Generator is ready when all data needed to serve are prepared.
     */
    bool ready() const;

    /** Prepares generator for serving.
     */
    void prepare();

protected:
    Generator(const boost::filesystem::path &root
              , const Resource &resource)
        : root_(root), resource_(resource)
    {}

    virtual bool ready_impl() const = 0;

    virtual void prepare_impl() const = 0;

    const boost::filesystem::path root_;
    const Resource resource_;
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

inline bool Generator::ready() const
{
    return ready_impl();
}

inline void Generator::prepare()
{
    prepare_impl();
}

#endif // mapproxy_generator_hpp_included_
