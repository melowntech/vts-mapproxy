#ifndef mapproxy_resourcebackend_mysql_hpp_included_
#define mapproxy_resourcebackend_mysql_hpp_included_

#include <boost/filesystem/path.hpp>

#include "utility/mysqldb.hpp"

#include "../resourcebackend.hpp"

namespace resource_backend {

class Mysql : public ResourceBackend {
public:
    struct Config {
        utility::mysql::Db::Parameters dbParams;
    };

    Mysql(const Config &config);

private:
    virtual Resource::map load_impl() const;

    const Config config_;
};

} // namespace resource_backend

#endif // mapproxy_resourcebackend_mysql_hpp_included_
