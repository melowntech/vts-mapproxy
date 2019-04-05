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

#include <map>

#include <boost/optional.hpp>
#include <boost/lexical_cast.hpp>

#include <tinyxml2.h>

#include "dbglog/dbglog.hpp"

#include "utility/format.hpp"

#include "vts-libs/registry/extensions.hpp"
#include "vts-libs/vts/csconvertor.hpp"
#include "vts-libs/vts/tileop.hpp"

#include "../error.hpp"
#include "../support/metatile.hpp"

#include "wmts.hpp"

namespace vr = vtslibs::registry;
namespace vre = vtslibs::registry::extensions;
namespace vts = vtslibs::vts;

namespace {

constexpr double wmtsResolution(0.28e-3);

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

std::string makeTemplate(const WmtsLayer &layer)
{
    std::ostringstream os;
    os << layer.rootPath;
    if (!layer.rootPath.empty() && layer.rootPath.back() != '/') {
        os << '/';
    }

    os << "{TileMatrix}-{TileCol}-{TileRow}." << layer.format;

    return os.str();
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
        .push("ows:ServiceContact").pop()
        ;
}

void point(Element &e, const char *name, const math::Point2 &p)
{
    e.text(name, utility::format("%.9f %.9f", p(0), p(1)));
}

void boundingBox(XML &x, const math::Extents2 &extents
                 , const boost::optional<std::string> &crs
                 = boost::none)
{
    Element bb(x, crs ? "ows:BoundingBox" : "ows:WGS84BoundingBox");
    bb.attribute("crs", crs ? crs->c_str() : "urn:ogc:def:crs:OGC:2:84");

    point(bb, "ows:LowerCorner", extents.ll);
    point(bb, "ows:UpperCorner", extents.ur);
}

template <typename T>
void widthAndHeight(Element &e, const char *widthName, const char *heightName
                    , const math::Size2_<T> &size)
{
    e.text(widthName, size.width);
    e.text(heightName, size.height);
}

struct TileMatrixSet {
    std::string id;
    std::string description;

    math::Extents2 extents;
    std::string srs;
    std::string extentsSrs;
    vts::LodRange lodRange;
    vts::TileRange tileRange;
    vre::Wmts wmts;
    double metersPerUnit;
    geo::SrsDefinition geographic;

    TileMatrixSet(const vr::ReferenceFrame &rf);

    math::Extents2 computeExtents(const Resource &r) const;
};

TileMatrixSet::TileMatrixSet(const vr::ReferenceFrame &rf)
    : id(rf.id), description(rf.description)
    , lodRange(vts::LodRange::emptyRange())
    , tileRange(math::InvalidExtents{})
    , metersPerUnit(1.0)
{
    if (const auto *ext = rf.findExtension<vre::Wmts>()) {
        wmts = *ext;
    } else {
        LOGTHROW(err1, Error)
            << "Reference frame <" << id << "> has no WMTS extension.";
    }

    if (wmts.content) {
        srs = *wmts.content;

        // this ... is here to silence compiler's "may-be-uninitialized"
        auto lod([]()->boost::optional<vts::Lod> { return boost::none; }());

        // find subtree-roots that provide data in "content" SRS
        for (const auto &item : rf.division.nodes) {
            const auto &node(item.second);
            if (node.srs != srs) { continue; }

            if (!lod) {
                lod = node.id.lod;
            } else if (*lod != node.id.lod) {
                LOGTHROW(err1, Error)
                    << "Malformed WMTS extension in reference frame <"
                    << id << ">: not all content root nodes are"
                    "at the same LOD.";
            }

            math::update(extents, node.extents);
            math::update(tileRange, node.id.x, node.id.y);
        }

        if (!lod) {
            LOGTHROW(err1, Error)
                << "Malformed WMTS extension in reference frame <"
                << id << ">: No root node found for wmts.content <"
                << srs << "> in the reference frame.";
        }
        lodRange = vts::LodRange(*lod);
    } else {
        if (rf.division.nodes.size() != 1) {
            LOGTHROW(err1, Error)
                << "Malformed WMTS extension in reference frame <"
                << id << ">: No wmts.content and the reference frame"
                "is not single-rooted.";
        }
        const auto &node(rf.division.nodes.begin()->second);
        extents = node.extents;
        srs = node.srs;
        lodRange = vts::LodRange(node.id.lod);
        tileRange = vts::tileRange(node.id);
    }

    extentsSrs = wmts.extentsSrs ? *wmts.extentsSrs : srs;
    extents = vts::CsConvertor(srs, extentsSrs)(extents);

    {
        const auto srsDef(vr::system.srs(extentsSrs).srsDef);
        geographic = srsDef.geographic();

        // get linear unit of given SRS (force computation for angluar systems)
        metersPerUnit = geo::linearUnit(srsDef, true);
    }
}

math::Extents2 TileMatrixSet::computeExtents(const Resource &r) const
{
    const vts::CsConvertor conv(srs, extentsSrs);

    math::Extents2 e(math::InvalidExtents{});

    // treat min lod as one gigantic metatile
    for (const auto &block : metatileBlocks
             (r, vts::TileId(r.lodRange.min, 0, 0), r.lodRange.min, false))
    {
        if (block.srs == srs) {
            math::update(e, conv(block.extents));
        }
    }

    return e;
}

void tileMatrix(XML &x, vts::Lod lod, const vts::TileRange &tr
                , const math::Extents2 &extents
                , const double metersPerUnit)
{
    Element e(x, "TileMatrix");

    const auto ts(vr::BoundLayer::tileSize());
    const auto trs(vts::tileRangesSize(tr));
    const auto es(math::size(extents));

    e.text("ows:Identifier", lod);

    // scale denominator:
    const auto pixelSize(es.width / (trs.width * ts.width));
    const auto scaleDenominator(pixelSize * metersPerUnit / wmtsResolution);
    e.text("ScaleDenominator", scaleDenominator);

    point(e, "TopLeftCorner", math::ul(extents));
    widthAndHeight(e, "TileWidth", "TileHeight", ts);
    widthAndHeight(e, "MatrixWidth", "MatrixHeight", trs);
}

/** Describe reference frame as tile matrix set
 */
void tileMatrixSet(XML &x, const TileMatrixSet &tms)
{
    Element e(x, "TileMatrixSet");

    e.text("ows:Identifier", tms.id);
    e.text("ows:Title", "VTS reference frame <" + tms.id + ">");
    e.text("ows:Abstract", tms.description);

    boundingBox(x, tms.extents, tms.wmts.projection);
    e.text("ows:SupportedCRS", tms.wmts.projection);
    if (tms.wmts.wellKnownScaleSet) {
        e.text("ows:WellKnownScaleSet", *tms.wmts.wellKnownScaleSet);
    }

    auto tr(tms.tileRange);
    for (const auto lod : tms.lodRange) {
        tileMatrix(x, lod, tr, tms.extents, tms.metersPerUnit);
        tr = vts::childRange(tr);
    }
}

void layerExtents(XML &x, const TileMatrixSet &tms, const Resource &r)
{
    const auto extents(tms.computeExtents(r));

    boundingBox(x, extents, tms.wmts.projection);
    boundingBox(x, vts::CsConvertor(tms.extentsSrs, tms.geographic)(extents));
}

void content(XML &x, const WmtsLayer::list &layers)
{
    typedef std::map<const vr::ReferenceFrame*, TileMatrixSet> map;
    map tmss;

    Element c(x, "Contents");
    for (const auto &layer : layers) {
        const auto &r(layer.resource);

        // remember reference frame
        auto ftmss(tmss.find(r.referenceFrame));
        if (ftmss == tmss.end()) {
            ftmss = tmss.insert
                (map::value_type
                 (r.referenceFrame, TileMatrixSet(*r.referenceFrame))).first;
        }
        auto &tms(ftmss->second);

        // update tile-matrix-set lod range
        update(tms.lodRange, r.lodRange.max);

        Element l(x, "Layer");
        l.text("ows:Title", "VTS Mapproxy resource <" + r.id.id + ">");
        l.text("ows:Abstract", r.comment);
        l.text("ows:Identifier", r.id.fullId());

        layerExtents(x, tms, r);

        const auto ct(contentType(layer.format));
        l.text("Format", ct);
        Element(x, "ResourceURL")
            .attribute("format", ct)
            .attribute("resourceType", "tile")
            .attribute("template", makeTemplate(layer))
            ;

        {
            Element s(x, "Style");
            s.attribute("isDefault", true);
            s.text("ows:Identifier", "default");
        }

        {
            // TileMatrixSet -> reference frame
            Element tmsl(x, "TileMatrixSetLink");
            tmsl.text("TileMatrixSet", r.id.referenceFrame);
        }
    }

    for (const auto &tms : tmss) { tileMatrixSet(x, tms.second); }
}

void operationsMetadata(XML &x, const WmtsResources &resources)
{
    Element om(x, "ows:OperationsMetadata");
    Element op(x, "ows:Operation");
    op.attribute("name", "GetCapabilities");

    Element dcp(x, "ows:DCP");
    Element http(x, "ows:HTTP");
    Element get(x, "ows:Get");
    get.attribute("xlink:href", resources.capabilitiesUrl);

    Element c(x, "ows:Constraint");
    Element aw(x, "ows:AllowedValues");
    aw.text("ows:Value", "REST");
}

} // namespace

std::string wmtsCapabilities(const WmtsResources &resources)
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
        operationsMetadata(x, resources);

        content(x, resources.layers);
    }

    return x.CStr();
}
