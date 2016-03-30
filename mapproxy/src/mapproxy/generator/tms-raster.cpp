#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "utility/premain.hpp"
#include "utility/raise.hpp"

#include "vts-libs/vts/io.hpp"

#include "../error.hpp"

#include "./tms-raster.hpp"
#include "./factory.hpp"

#include "browser2d/index.html.hpp"

namespace vr = vadstena::registry;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const Generator::Config &config
                                      , const Resource &resource)
    {
        return std::make_shared<TmsRaster>(config, resource);
    }

private:
    static utility::PreMain register_;
};

utility::PreMain Factory::register_([]()
{
    Generator::registerType
        (resdef::TmsRaster::generator, std::make_shared<Factory>());
});

} // namespace

TmsRaster::TmsRaster(const Config &config, const Resource &resource)
    : Generator(config, resource)
    , definition_(this->resource().definition<resdef::TmsRaster>())
{
    // TODO: check datasets
    makeReady();
}

void TmsRaster::prepare_impl()
{
    LOG(info2) << "No need to prepare.";
}

vts::MapConfig TmsRaster::mapConfig_impl(const std::string &referenceFrame
                                         , ResourceRoot root)
    const
{
    const auto &res(resource());
    auto frfd(res.referenceFrames.find(referenceFrame));
    if (frfd == res.referenceFrames.end()) {
        LOGTHROW(err1, NotFound)
            << "Reference frame <" << referenceFrame << "> is not handed by "
            << "this resource generator.";
    }
    const auto &rf(frfd->second);

    vts::MapConfig mapConfig;
    mapConfig.referenceFrame = *rf.referenceFrame;

    // this is Tiled service: we have bound layer only
    vr::BoundLayer bl;
    bl.id = res.id.fullId();
    bl.numericId = 0; // no numeric ID
    bl.type = vr::BoundLayer::Type::raster;

    // build url
    bl.url = prependRoot
        (str(boost::format("{lod}-{x}-{y}.%s") % definition_.format)
         , resource(), referenceFrame, root);

    bl.lodRange = rf.lodRange;
    bl.tileRange = rf.tileRange;
    bl.credits = res.credits;
    mapConfig.boundLayers.add(bl);

    return mapConfig;
}

Generator::Task TmsRaster::generateFile_impl(const FileInfo &fileInfo
                                             , const Sink::pointer &sink) const
{
    TmsFileInfo fi(fileInfo, config().fileFlags);

    switch (fi.type) {
    case TmsFileInfo::Type::unknown:
        sink->error(utility::makeError<NotFound>("Unrecognized filename."));
        return {};

    case TmsFileInfo::Type::config: {
        std::ostringstream os;
        mapConfig(os, fileInfo.referenceFrame, ResourceRoot::none);
        sink->content(os.str(), fi.sinkFileInfo());
        return {};
    };

    case TmsFileInfo::Type::support:
        LOG(info4) << "Size: " << fi.support->size << " vs "
                   << sizeof(browser2d::index_html);
        sink->content(fi.support->data, fi.support->size
                      , fi.sinkFileInfo(), false);
        return {};

    case TmsFileInfo::Type::imagery: {
        if (fi.format != definition_.format) {
            sink->error(utility::makeError<NotFound>
                        ("Format <%s> is not supported by this resource (%s)."
                         , fi.format, definition_.format));
            return {};
        }
        break;
    }

    case TmsFileInfo::Type::mask:
        // TODO: implement me
        break;
    }

    return {};
}

} // namespace generator
