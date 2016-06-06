#ifndef mapproxy_support_coverage_hpp_included_
#define mapproxy_support_coverage_hpp_included_


#include "vts-libs/vts/nodeinfo.hpp"
#include "imgproc/rastermask/mappedqtree.hpp"

namespace vts = vadstena::vts;

typedef imgproc::mappedqtree::RasterMask MaskTree;

vts::NodeInfo::CoverageMask
generateCoverage(const int size, const vts::NodeInfo &nodeInfo
                 , const imgproc::mappedqtree::RasterMask &maskTree
                 , vts::NodeInfo::CoverageType type
                 = vts::NodeInfo::CoverageType::pixel);

#endif // mapproxy_support_coverage_hpp_included_
