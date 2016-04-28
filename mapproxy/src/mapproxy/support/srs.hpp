#ifndef mapproxy_support_srs_hpp_included_
#define mapproxy_support_srs_hpp_included_

#include <boost/optional.hpp>

#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/csconvertor.hpp"

namespace vts = vadstena::vts;

vts::CsConvertor sds2phys(const vts::NodeInfo &nodeInfo
                          , const boost::optional<std::string> &geoidGrid);

vts::CsConvertor sds2nav(const vts::NodeInfo &nodeInfo
                          , const boost::optional<std::string> &geoidGrid);

#endif // mapproxy_support_srs_hpp_included_
