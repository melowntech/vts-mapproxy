#ifndef mapproxy_support_srs_hpp_included_
#define mapproxy_support_srs_hpp_included_

#include <boost/optional.hpp>

#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/csconvertor.hpp"

namespace vts = vtslibs::vts;

vts::CsConvertor sds2phys(const vts::NodeInfo &nodeInfo
                          , const boost::optional<std::string> &geoidGrid);

vts::CsConvertor sds2nav(const vts::NodeInfo &nodeInfo
                         , const boost::optional<std::string> &geoidGrid);

geo::SrsDefinition sds(const vts::NodeInfo &nodeInfo
                       , const boost::optional<std::string> &geoidGrid);

/** Convertor between geoid-shifted SDS to raw SDS
 *  Returns dummy convertor if no grid applies.
 */
vts::CsConvertor sdsg2sdsr(const vts::NodeInfo &nodeInfo
                           , const boost::optional<std::string> &geoidGrid);

/** Convertor between physical coordinates and node's SDS.
 */
vts::CsConvertor phys2sds(const vts::NodeInfo &nodeInfo);

#endif // mapproxy_support_srs_hpp_included_
