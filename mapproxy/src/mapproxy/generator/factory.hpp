#ifndef mapproxy_generator_factory_hpp_included_
#define mapproxy_generator_factory_hpp_included_

#include <boost/any.hpp>

#include "../generator.hpp"

namespace vr = vtslibs::registry;

struct Generator::Factory {
    typedef std::shared_ptr<Factory> pointer;
    virtual ~Factory() {}
    virtual Generator::pointer create(const Params &params) = 0;

    /** Parses configuration from any value (can be json or python object).
     *  Throws in case of error.
     */
    virtual DefinitionBase::pointer definition() = 0;

    /** If true is returned a default system resource is generated for each
     *  reference frame.
     */
    virtual bool systemInstance() const { return false; }
};

#endif // mapproxy_generator_factory_hpp_included_
