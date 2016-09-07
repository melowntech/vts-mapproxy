#ifndef mapproxy_support_demconfig_hpp_included_
#define mapproxy_support_demconfig_hpp_included_

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

struct DemConfig {
    boost::optional<double> effectiveGSD;

    DemConfig() {}
};

DemConfig loadDemConfig(const boost::filesystem::path &path
                        , bool ignoreMissing = false);

#endif // mapproxy_support_demconfig_hpp_included_
