/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef mapproxy_support_coverage_hpp_included_
#define mapproxy_support_coverage_hpp_included_


#include "vts-libs/vts/nodeinfo.hpp"
#include "imgproc/rastermask/mappedqtree.hpp"

namespace vts = vtslibs::vts;

typedef imgproc::mappedqtree::RasterMask MaskTree;

/** Complex converage, used for surface mask.
 */
vts::NodeInfo::CoverageMask
generateCoverage(const int size, const vts::NodeInfo &nodeInfo
                 , const imgproc::mappedqtree::RasterMask &maskTree
                 , vts::NodeInfo::CoverageType type
                 = vts::NodeInfo::CoverageType::pixel);

/** Boundlayer mask, used for TMS mask.
 */
cv::Mat boundlayerMask(const vts::TileId &tileId
                       , const imgproc::mappedqtree::RasterMask &maskTree);

#endif // mapproxy_support_coverage_hpp_included_
