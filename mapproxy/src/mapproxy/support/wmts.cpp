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

#include <set>

#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>

#include <tinyxml2.h>

#include "dbglog/dbglog.hpp"

#include "utility/format.hpp"

#include "vts-libs/registry/extensions.hpp"
#include "vts-libs/vts/csconvertor.hpp"

#include "../error.hpp"

#include "wmts.hpp"

namespace vr = vtslibs::registry;
namespace vre = vtslibs::registry::extensions;
namespace vts = vtslibs::vts;

namespace {

typedef tinyxml2::XMLPrinter XML;

template <typename T> T&& extract(T &&value) { return std::forward<T>(value); }
const char* extract(const std::string &value) { return value.c_str(); }
const char* extract(std::string &&value) { return value.c_str(); }

/** Stack-based XML element writer.
 *
 *  Can be used independently or combined with nested elements via push/pop
 *  pair.
 */
class Element {
public:
    Element(XML &x, const char *name)
        : x_(x), closed_(false), parent_()
    {
        x_.OpenElement(name);
    }

    ~Element() { close(); }

    template <typename T>
    Element& attribute(const char *name, T &&value) {
        x_.PushAttribute(name, extract(std::forward<T>(value)));
        return *this;
    }

    template <typename T>
    Element& text(T &&value) {
        x_.PushText(extract(std::forward<T>(value)));
        return *this;
    }

    /** Add element with single text
     */
    template <typename T>
    Element& text(const char *element, T &&value) {
        Element(x_, element).text(std::forward<T>(value));
        return *this;
    }

    /** Push nested element. Must be pop()ed to work properly.
     *  NB: Return by value!
     */
    Element push(const char *name) { return Element(this, name); }

    /** Pop from nested element.
     */
    Element& pop() { close(); return *parent_; }

private:
    void close() { if (!closed_) { x_.CloseElement(); closed_ = true; } }

    Element(Element *parent, const char *name)
        : x_(parent->x_), closed_(false), parent_(parent)
    {
        x_.OpenElement(name);
    }

    XML &x_;
    bool closed_;
    Element *parent_;
};

template <typename T>
void text(XML &x, const char *element, T &&value) {
    Element(x, element).text(std::forward<T>(value));
}

/** TODO: make configurable
 */
void serviceIdentification(XML &x)
{
    Element(x, "ows:ServiceIdentification")
        .text("ows:Title", "VTS Mapproxy")
        .text("ows:ServiceType", "OGC WMTS")
        .text("ows:ServiceTypeVersion", "1.3.0")
        ;
}

/** TODO: make configurable
 */
void serviceProvider(XML &x)
{
    Element(x, "ows:ServiceProvider")
        .text("ows:ProviderName", "Mapproxy")
        .text("ows:ProviderSite", "https://melown.com")
        .text("ows:ServiceContact")
        ;
}

typedef std::set<const vr::ReferenceFrame*> RfSet;

void boundingBox(XML &x, const math::Extents2 &extents
                 , const boost::optional<std::string> &crs
                 = boost::none)
{
    Element bb(x, crs ? "ows:BoundingBox" : "ows:WGS84BoundingBox");
    bb.attribute("crs", crs ? crs->c_str() : "urn:ogc:def:crs:OGC:2:84");

    // TODO: use custom format
    bb.text("ows:LowerCorner"
            , utility::format("%.9f %.9f", extents.ll(0), extents.ll(1)));
    bb.text("ows:UpperCorner"
            , utility::format("%.9f %.9f", extents.ur(0), extents.ur(1)));
}

RfSet content(XML &x, const WmtsLayer::list &layers)
{
    RfSet rfs;

    Element c(x, "Contents");
    for (const auto &layer : layers) {
        const auto &r(layer.resource);

        Element l(x, "Layer");
        l.text("ows:Title", "VTS Mapproxy resource <" + r.id.id + ">");
        l.text("ows:Abstract", r.comment);
        Element(x, "ows::Keywords");
        l.text("ows:Identifier", r.id.fullId());

        l.text("Format", contentType(layer.format));

        // TODO: extents from reference frame
        //  <ows:WGS84BoundingBox>
        //  <ows:BoundingBox>
#if 0
        const auto wgs84Extents
            (vts::CsConvertor(contentSrs, vr::system.srs
                              (contentSrs).srsDef.geographic())(rfExtents));
        boundingBox(x, wgs84Extents);
#endif

        {
            Element s(x, "Style");
            s.attribute("isDefault", true);
            s.text("ows:Identifier", "default");
        }

        {
            // TileMatrixSet -> reference frame
            Element tmsl(x, "TileMatrixSetLink");
            tmsl.text("TileMatrixSet", r.id.referenceFrame);

            // TODO: TileMatrixSetLimits from resource (tile/lod range)
        }

        // remember reference frame
        rfs.insert(r.referenceFrame);
    }

    return rfs;
}

void tileMatrixSet(XML &x, const vr::ReferenceFrame &rf)
{
    const auto *wmts(rf.findExtension<vre::Wmts>());

    Element e(x, "TileMatrixSet");

    e.text("ows:Title", "VTS reference frame <" + rf.id + ">");
    e.text("ows:Abstract", rf.description);

    const auto &physicalSrs(wmts->physicalSrs ? *wmts->physicalSrs
                            : rf.model.physicalSrs);

    // compute extents in reference frame SRS
    math::Extents2 rfExtents(math::InvalidExtents{});

    std::string contentSrs;

    if (wmts->content) {
        // find subtree-roots that provide data in "content" SRS
        for (const auto &item : rf.division.nodes) {
            const auto &node(item.second);
            if (node.srs != wmts->content) { continue; }

            math::update(rfExtents, node.extents);
        }
        if (!math::valid(rfExtents)) {
            LOGTHROW(err1, Error)
                << "Malformed WMTS extension in reference frame <"
                << rf.id << ">: No root node found for wmts.content <"
                << *wmts->content << "> in the reference frame.";
        }
        contentSrs = *wmts->content;
    } else {
        if (rf.division.nodes.size() != 1) {
            LOGTHROW(err1, Error)
                << "Malformed WMTS extension in reference frame <"
                << rf.id << ">: No wmts.content and the reference frame"
                "is not single-rooted.";
        }
        const auto &node(rf.division.nodes.begin()->second);
        rfExtents = node.extents;
        contentSrs = node.srs;
    }

    const auto extents
        (vts::CsConvertor(contentSrs, physicalSrs)(rfExtents));

    boundingBox(x, extents, wmts->projection);
    e.text("ows:SupportedCRS", wmts->projection);
    if (wmts->wellKnownScaleSet) {
        e.text("ows:WellKnownScaleSet", *wmts->wellKnownScaleSet);
    }
}

void matrixSets(XML &x, const RfSet &rfs)
{
    for (const auto *rf : rfs) { tileMatrixSet(x, *rf); }
}

} // namespace

std::string wmtsCapabilities(const WmtsLayer::list &layers)
{
    XML x;

    // xml.PushHeader(false, false);
    x.PushDeclaration(R"(xml version="1.0" encoding="UTF-8")");

    {
        Element e(x, "Capabilities");
        e.attribute("version", "1.3.0")
            .attribute("xmlns", "http://www.opengis.net/wmts/1.0")
            .attribute("xmlns:gml", "http://www.opengis.net/gml")
            .attribute("xmlns:ows", "http://www.opengis.net/ows/1.1")
            .attribute("xmlns:xlink", "http://www.w3.org/1999/xlink")
            .attribute("xmlns:xsi"
                       , "http://www.w3.org/2001/XMLSchema-instance")
            .attribute("xsi:schemaLocation"
                       , "http://www.opengis.net/wmts/1.0 "
                       "http://schemas.opengis.net/wmts/1.0/"
                       "wmtsGetCapabilities_response.xsd")
            ;

        serviceIdentification(x);
        serviceProvider(x);

        matrixSets(x, content(x, layers));
    }

    return x.CStr();
}
