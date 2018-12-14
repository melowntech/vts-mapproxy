/**
 * Copyright (c) 2018 Melown Technologies SE
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

#ifndef mapproxy_definition_tms_hpp_included_
#define mapproxy_definition_tms_hpp_included_

#include <boost/optional.hpp>
#include <boost/filesystem.hpp>

#include "geo/geodataset.hpp"

#include "../resource.hpp"

// fwd
namespace Json { class Value; }
namespace boost { namespace python { class dict; } }

namespace resource {

// raster formats

struct TmsRasterSynthetic : public DefinitionBase {
    boost::optional<std::string> mask;
    RasterFormat format;

    TmsRasterSynthetic(): format(RasterFormat::jpg) {}

    static constexpr Resource::Generator::Type type
        = Resource::Generator::Type::tms;

    void parse(const Json::Value &value);
    void build(Json::Value &value) const;
    void parse(const boost::python::dict &value);

protected:
    virtual Changed changed_impl(const DefinitionBase &other) const;
};

struct TmsRasterPatchwork : public TmsRasterSynthetic {
    TmsRasterPatchwork() = default;

    static constexpr char driverName[] = "tms-raster-patchwork";

private:
    virtual void from_impl(const boost::any &value);
    virtual void to_impl(boost::any &value) const;
    virtual Changed changed_impl(const DefinitionBase &other) const;
    virtual bool frozenCredits_impl() const { return false; }
};

struct TmsRasterSolid : public TmsRasterSynthetic {
    cv::Vec3b color;

    TmsRasterSolid() : color(0xff, 0xff, 0xff) {}

    static constexpr char driverName[] = "tms-raster-solid";

private:
    virtual void from_impl(const boost::any &value);
    virtual void to_impl(boost::any &value) const;
    virtual Changed changed_impl(const DefinitionBase &other) const;
    virtual bool frozenCredits_impl() const { return false; }
};

struct TmsRaster : public DefinitionBase {
    std::string dataset;
    boost::optional<std::string> mask;
    RasterFormat format;
    bool transparent;
    boost::optional<geo::GeoDataset::Resampling> resampling;

    TmsRaster(): format(RasterFormat::jpg), transparent(false) {}

    static constexpr Resource::Generator::Type type
        = Resource::Generator::Type::tms;
    static constexpr char driverName[] = "tms-raster";

protected:
    virtual void from_impl(const boost::any &value);
    virtual void to_impl(boost::any &value) const;
    virtual Changed changed_impl(const DefinitionBase &other) const;
    virtual bool frozenCredits_impl() const { return false; }
};

struct TmsRasterRemote : public DefinitionBase {
    std::string remoteUrl;
    boost::optional<boost::filesystem::path> mask;

    TmsRasterRemote() {}

    static constexpr Resource::Generator::Type type
        = Resource::Generator::Type::tms;
    static constexpr char driverName[] = "tms-raster-remote";

private:
    virtual void from_impl(const boost::any &value);
    virtual void to_impl(boost::any &value) const;
    virtual Changed changed_impl(const DefinitionBase &other) const;
    virtual bool frozenCredits_impl() const { return false; }
};

struct TmsBing : public DefinitionBase {
    std::string metadataUrl;

    TmsBing() {}

    static constexpr Resource::Generator::Type type
        = Resource::Generator::Type::tms;
    static constexpr char driverName[] = "tms-bing";

private:
    virtual void from_impl(const boost::any &value);
    virtual void to_impl(boost::any &value) const;
    virtual Changed changed_impl(const DefinitionBase &other) const;
    virtual bool frozenCredits_impl() const { return false; }
};

struct TmsWindyty : public TmsRaster {
    int forecastOffset;

    TmsWindyty() : forecastOffset() {}

    static constexpr Resource::Generator::Type type
        = Resource::Generator::Type::tms;
    static constexpr char driverName[] = "tms-windyty";

private:
    virtual void from_impl(const boost::any &value);
    virtual void to_impl(boost::any &value) const;
    virtual Changed changed_impl(const DefinitionBase &other) const;
    virtual bool frozenCredits_impl() const { return false; }
};

} // namespace resource

#endif // mapproxy_definition_tms_hpp_included_
