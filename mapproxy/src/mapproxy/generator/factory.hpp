#ifndef mapproxy_generator_factory_hpp_included_
#define mapproxy_generator_factory_hpp_included_

#include "../generator.hpp"

struct Generator::Factory {
    typedef std::shared_ptr<Factory> pointer;
    virtual ~Factory() {}
    virtual Generator::pointer create(const Resource &resource) = 0;
};

#endif // mapproxy_generator_factory_hpp_included_
