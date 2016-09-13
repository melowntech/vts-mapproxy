#ifndef mapproxy_resourcebackend_python_hpp_included_
#define mapproxy_resourcebackend_python_hpp_included_

#include <mutex>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/filesystem/path.hpp>

#include "../resourcebackend.hpp"

namespace python = boost::python;

namespace resource_backend {

class Python : public ResourceBackend {
public:
    struct Config {
        typedef std::map<std::string, std::string> Options;
        boost::filesystem::path script;
        Options options;
    };

    Python(const GenericConfig &genericConfig, const Config &config);

private:
    virtual Resource::map load_impl() const;

    virtual void error_impl(const Resource::Id &resourceId
                            , const std::string &message) const;

    void errorRaw(const Resource::Id &resourceId
                  , const std::string &message) const;

    mutable std::mutex mutex_;
    python::object run_;
    python::object error_;
};

} // namespace resource_backend

#endif // mapproxy_resourcebackend_python_hpp_included_
