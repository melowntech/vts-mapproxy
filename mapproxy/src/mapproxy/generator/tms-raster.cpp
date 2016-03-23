#include <boost/filesystem.hpp>

#include "utility/premain.hpp"

#include "./tms-raster.hpp"
#include "./factory.hpp"

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

} // namespace generator
