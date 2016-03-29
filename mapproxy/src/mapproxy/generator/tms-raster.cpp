#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "utility/premain.hpp"

#include "../error.hpp"

#include "./tms-raster.hpp"
#include "./factory.hpp"

namespace vr = vadstena::registry;

namespace generator {

namespace {

struct Factory : Generator::Factory {
    virtual Generator::pointer create(const boost::filesystem::path &root
                                      , const Resource &resource)
    {
        return std::make_shared<TmsRaster>(root, resource);
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

TmsRaster::TmsRaster(const boost::filesystem::path &root
                     , const Resource &resource)
    : Generator(root, resource)
    , definition_(boost::any_cast<resdef::TmsRaster>(resource.definition))
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

    bl.url = prependRoot
        (str(boost::format("{lod}-{x}-{y}.%s") % definition_.format)
         , resource(), referenceFrame, root);

    bl.lodRange = rf.lodRange;
    bl.tileRange = rf.tileRange;
    bl.credits = res.credits;
    mapConfig.boundLayers.add(bl);

    return mapConfig;
}

} // namespace generator
