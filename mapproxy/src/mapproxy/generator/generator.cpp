#include "dbglog/dbglog.hpp"

#include "../error.hpp"
#include "../generator.hpp"
#include "./factory.hpp"

namespace {

typedef std::map<std::string, Generator::Factory::pointer> Registry;
Registry registry;

Generator::Factory::pointer findFactory(const std::string &type)
{
    auto fregistry(registry.find(type));
    if (fregistry == registry.end()) {
        LOGTHROW(err1, UnknownGenerator)
            << "Unknown resource backend <" << type << ">.";
    }
    return fregistry->second;
}

} // namespace

void Generator::registerType(const std::string &type
                             , const Factory::pointer &factory)
{
    registry.insert(Registry::value_type(type, factory));
}

Generator::pointer Generator::create(const std::string &type
                                     , const Resource &resource)
{
    try {
        return findFactory(type)->create(resource);
    } catch (const boost::bad_any_cast&) {
        LOGTHROW(err2, InvalidConfiguration)
            << "Passed resource does not match generator <"
            << type << ">.";
    }
    throw;
}
