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

#include <tinyxml2.h>

#include "wmts.hpp"

namespace {

/** Stack-based XML element writer.
 *
 *  Can be used independently or combined with nested elements via push/pop
 *  pair.
 */
class Element {
public:
    Element(tinyxml2::XMLPrinter &x, const char *name)
        : x_(x), closed_(false), parent_()
    {
        x_.OpenElement(name);
    }

    ~Element() { close(); }

    template <typename T>
    Element& attribute(const char *name, T &&value) {
        x_.PushAttribute(name, std::forward<T>(value));
        return *this;
    }

    Element& attribute(const char *name, const std::string &value) {
        x_.PushAttribute(name, value.c_str());
        return *this;
    }

    template <typename T>
    Element& text(T &&value) {
        x_.PushText(std::forward<T>(value));
        return *this;
    }

    Element& text(const std::string &value) { return text(value.c_str()); }

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

    tinyxml2::XMLPrinter &x_;
    bool closed_;
    Element *parent_;
};

/** TODO: make configurable
 */
void serviceIdentification(tinyxml2::XMLPrinter &x)
{
    Element(x, "ows:ServiceIdentification")
        .push("ows:Title").text("VTS Mapproxy").pop()
        .push("ows:ServiceType").text("OGC WMTS").pop()
        .push("ows:ServiceTypeVersion").text("1.3.0").pop()
        ;
}

/** TODO: make configurable
 */
void serviceProvider(tinyxml2::XMLPrinter &x)
{
    Element(x, "ows:ServiceProvider")
        .push("ows:ProviderName").text("Mapproxy").pop()
        .push("ows:ProviderSite").text("https://melown.com").pop()
        .push("ows:ServiceContact").pop()
        ;
}

} // namespace

std::string wmtsCapabilities(const Resource &resources)
{
    (void) resources;

    tinyxml2::XMLPrinter x;

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
    }

    return x.CStr();
}
