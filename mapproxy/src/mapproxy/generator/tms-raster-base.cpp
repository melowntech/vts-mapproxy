/**
 * Copyright (c) 2019 Melown Technologies SE
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

#include "utility/raise.hpp"
#include "utility/httpquery.hpp"

#include "../support/wmts.hpp"

#include "tms-raster-base.hpp"

namespace uq = utility::query;

namespace generator {

namespace {
Generator::Properties
wmtsSupport(const Generator::Params &params
            , const boost::optional<RasterFormat> &format)
{
    Generator::Properties props;
    if (!format) { return props; }
    if (!params.resource.referenceFrame->findExtension<vre::Wmts>()) {
        return props;
    }
    return props.support(GeneratorInterface::Interface::wmts);
}

} // namespace

TmsRasterBase
::TmsRasterBase(const Params &params
                , const boost::optional<RasterFormat> &format)
    : Generator(params, wmtsSupport(params, format))
    , format_(format ? *format : RasterFormat())
    , wmts_(properties().isSupported(GeneratorInterface::Interface::wmts)
            ? params.resource.referenceFrame->findExtension<vre::Wmts>()
            : nullptr)
{}

Generator::Task TmsRasterBase
::generateFile_impl(const FileInfo &fileInfo, Sink &sink) const
{
    // handle special case
    switch (fileInfo.interface.interface) {
    case GeneratorInterface::Interface::vts:
        return generateVtsFile_impl(fileInfo, sink);

    case GeneratorInterface::Interface::wmts:
        return wmtsInterface(fileInfo, sink);

    default:
        sink.error(utility::makeError<InternalError>
                   ("TMS resource has no <%s> interface."
                    , fileInfo.interface));
        return {};
    }
}

const vre::Wmts& TmsRasterBase::getWmts() const
{
    if (!wmts_) {
        utility::raise<NotFound>
            ("WMTS interface disabled, no <wmts> extension in "
             "reference frame <%s> or not supported by <%s> driver."
             , referenceFrameId(), resource().generator);
    }

    return *wmts_;
}

wmts::WmtsResources TmsRasterBase::wmtsResources(const WmtsFileInfo &fileInfo)
    const
{
    const auto &fi(fileInfo.fileInfo);
    bool introspection
        (!uq::empty(uq::find(uq::splitQuery(fi.query), "is")));

    wmts::WmtsResources resources;

    resources.layers.emplace_back(resource());
    auto &layer(resources.layers.back());

    layer.format = format_;

    if (!introspection && !config().externalUrl) {
        LOG(warn2)
            << "External URL unset in configuration, cannot generate "
            "proper URL template in WMTSCapabilities; please, set "
            "external URL. Switching to introspection mode.";
        introspection = true;
    }

    // build root path
    if (introspection) {
        // used in introspection -> local, can be relative from wmts interface
        // to vts interface
        const auto resdiff
            (resolveRoot
             (id(), fi.interface
              , id(), fi.interface.as(GeneratorInterface::Interface::vts)));

        layer.rootPath =
            prependRoot(std::string(), resource(), resdiff);

        resources.capabilitiesUrl = "./" + fileInfo.capabilitesName;
    } else {
        layer.rootPath = *config().externalUrl
            + prependRoot(std::string(), resource()
                          , ResourceRoot::Depth::referenceFrame);
        resources.capabilitiesUrl =
            layer.rootPath + "/" + fileInfo.capabilitesName;
    }

    return resources;
}

Generator::Task TmsRasterBase
::wmtsInterface(const FileInfo &fileInfo, Sink &sink) const
{
    WmtsFileInfo fi(fileInfo);

    switch (fi.type) {
    case WmtsFileInfo::Type::unknown:
        sink.error(utility::makeError<NotFound>("Unrecognized filename."));
        break;

    case WmtsFileInfo::Type::capabilities:
        sink.content(wmtsCapabilities(wmtsResources(fi)), fi.sinkFileInfo());
        return {};

    case WmtsFileInfo::Type::support:
        supportFile(*fi.support, sink, fi.sinkFileInfo());
        break;

    case WmtsFileInfo::Type::listing:
        sink.listing(fi.listing);
        break;

    default:
        sink.error(utility::makeError<InternalError>
                    ("Not implemented yet."));
    }

    return {};
}

} // namespace generator
