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
#include "utility/format.hpp"
#include "utility/httpquery.hpp"

#include "../support/wmts.hpp"

#include "files.hpp"

#include "tms-raster-base.hpp"
#include "providers.hpp"

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

class AtlasProvider
    : public Generator::Provider
    , public VtsAtlasProvider
{
public:
    AtlasProvider(TmsRasterBase &tms)
        : tms_(tms)
    {}

private:
    Generator::Task atlas_impl(const vts::TileId &tileId, Sink&
                               , const Sink::FileInfo &sfi
                               , bool atlas) const override
    {
        TmsRasterBase::ImageFlags imageFlags;
        imageFlags.forceFormat = true;
        imageFlags.atlas = atlas;
        return[=](Sink &sink, Arsenal &arsenal) {
            tms_.generateTileImage(tileId, sfi, RasterFormat::jpg
                                   , sink, arsenal, imageFlags);
        };
    }

    TmsRasterBase &tms_;
};

TmsRasterBase
::TmsRasterBase(const Params &params
                , const boost::optional<RasterFormat> &format)
    : Generator(params, wmtsSupport(params, format))
    , format_(format ? *format : RasterFormat())
    , wmts_(properties().isSupported(GeneratorInterface::Interface::wmts)
            ? params.resource.referenceFrame->findExtension<vre::Wmts>()
            : nullptr)
{
    setProvider(std::make_unique<AtlasProvider>(*this));
}

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

    // build root path
    if (introspection) {
        // used in introspection -> local, can be relative from wmts interface
        layer.rootPath = "./";

        resources.capabilitiesUrl = "./" + fileInfo.capabilitesName;
    } else {
        layer.rootPath = config().externalUrl
            + prependRoot(std::string(), id()
                          , {type(), GeneratorInterface::Interface::wmts}
                          , ResourceRoot::Depth::referenceFrame);
        resources.capabilitiesUrl =
            layer.rootPath + "/" + fileInfo.capabilitesName;
    }

    return resources;
}

std::string TmsRasterBase::wmtsReadme() const
{
    vs::SupportFile::Vars vars;
    vars["externalUrl"] = config().externalUrl;
    vars["url"] = url(GeneratorInterface::Interface::wmts);

    return files::wmtsReadme.expand(&vars, nullptr);
}

Generator::Task TmsRasterBase
::wmtsInterface(const FileInfo &fileInfo, Sink &sink) const
{
    WmtsFileInfo fi(fileInfo);

    switch (fi.type) {
    case WmtsFileInfo::Type::unknown:
        sink.error(utility::makeError<NotFound>("Unrecognized filename."));
        break;

    case WmtsFileInfo::Type::image:
        return [=](Sink &sink, Arsenal &arsenal) {
            ImageFlags imageFlags;
            imageFlags.dontOptimize = true;
            generateTileImage(fi.tileId, fi.sinkFileInfo(), fi.format
                              , sink, arsenal, imageFlags);
        };

    case WmtsFileInfo::Type::capabilities:
        sink.content(wmtsCapabilities(wmtsResources(fi)), fi.sinkFileInfo());
        return {};

    case WmtsFileInfo::Type::support:
        supportFile(*fi.support, sink, fi.sinkFileInfo());
        break;

    case WmtsFileInfo::Type::listing:
        sink.listing(fi.listing, "", markdown(wmtsReadme()));
        break;

    case WmtsFileInfo::Type::readme:
        sink.markdown(utility::format("%s: WMTS Readme", id().fullId())
                      , wmtsReadme());
        break;

    default:
        sink.error(utility::makeError<InternalError>
                    ("Not implemented yet."));
    }

    return {};
}

} // namespace generator
