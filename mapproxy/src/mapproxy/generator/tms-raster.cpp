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
        ({"tms", "tms-raster"}, std::make_shared<Factory>());
});

} // namespace

TmsRaster::TmsRaster(const boost::filesystem::path &root
                     , const Resource &resource)
    : Generator(root, resource)
{
    // TODO: remove if new and fails to run
    if (create_directories(this->root())) {
        // new dataset
    } else {
        // reopen of existing dataset
    }

    makeReady();
}

void TmsRaster::prepare_impl()
{
    LOG(info4) << "Preparing.";
}

} // namespace generator
