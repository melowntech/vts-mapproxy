#ifndef mapproxy_resourcebackend_python_hpp_included_
#define mapproxy_resourcebackend_python_hpp_included_

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/filesystem/path.hpp>

#include "../resourcebackend.hpp"

namespace python = boost::python;

namespace resource_backend {

class Python : public ResourceBackend {
public:
    struct Config {
        boost::filesystem::path script;
    };

    Python(const Config &config);

private:
    virtual Resource::map load_impl() const;

    python::object run_;
};

} // namespace resource_backend

#endif // mapproxy_resourcebackend_python_hpp_included_
