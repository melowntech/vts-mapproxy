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
