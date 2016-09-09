#ifndef mapproxy_resourcebackend_conffile_hpp_included_
#define mapproxy_resourcebackend_conffile_hpp_included_

#include <boost/filesystem/path.hpp>

#include "../resourcebackend.hpp"

namespace resource_backend {

class Conffile : public ResourceBackend {
public:
    struct Config {
        boost::filesystem::path path;
    };

    Conffile(const GenericConfig &genericConfig
             , const Config &config);

private:
    virtual Resource::map load_impl() const;

    const Config config_;
};

} // namespace resource_backend

#endif // mapproxy_resourcebackend_conffile_hpp_included_
