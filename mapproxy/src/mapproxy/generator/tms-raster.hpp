#ifndef mapproxy_generator_tms_factory_hpp_included_
#define mapproxy_generator_tms_factory_hpp_included_

#include "../generator.hpp"

namespace generator {

class TmsRaster : public Generator {
public:
    TmsRaster(const boost::filesystem::path &root
              , const Resource &resource);

private:
    virtual void prepare_impl();
};

} // namespace generator

#endif // mapproxy_generator_tms_factory_hpp_included_
