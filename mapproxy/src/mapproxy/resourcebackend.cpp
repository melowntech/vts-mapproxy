#include "dbglog/dbglog.hpp"

#include "./error.hpp"
#include "./resourcebackend.hpp"
#include "./resourcebackend/factory.hpp"

namespace po = boost::program_options;

namespace {

typedef std::map<std::string, ResourceBackend::Factory::pointer> Registry;
Registry registry;

ResourceBackend::Factory::pointer findFactory(const std::string &type)
{
    auto fregistry(registry.find(type));
    if (fregistry == registry.end()) {
        LOGTHROW(err2, UnknownResourceBackend)
            << "Unknown resource backend <" << type << ">.";
    }
    return fregistry->second;
}

} // namespace

void ResourceBackend::registerType(const std::string &type
                                   , const Factory::pointer &factory)
{
    registry.insert(Registry::value_type(type, factory));
}

service::UnrecognizedParser::optional
ResourceBackend::configure(const std::string &prefix, const std::string &type
                           , boost::any &config)
{
    return findFactory(type)->configure(prefix, config);
}

void ResourceBackend::printConfig(std::ostream &os, const std::string &prefix
                                  , const std::string &type
                                  , const boost::any &config)
{
    os << prefix << "type = " << type << "\n";
    return findFactory(type)->printConfig(os, prefix, config);
}

std::vector<std::string> ResourceBackend::listTypes(const std::string &prefix)
{
    std::vector<std::string> out;
    for (const auto &ritem : registry) {
        out.push_back(prefix + ritem.first);
    }
    return out;
}

ResourceBackend::pointer
ResourceBackend::create(const std::string &type, const boost::any &config)
{
    try {
        return findFactory(type)->create(config);
    } catch (const boost::bad_any_cast&) {
        LOGTHROW(err2, InvalidConfiguration)
            << "Passed configuration does not match resource backend <"
            << type << ">.";
    }
    throw;
}
