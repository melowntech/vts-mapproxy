#ifndef mapproxy_generator_tms_windyty_hpp_included_
#define mapproxy_generator_tms_windyty_hpp_included_

#include <mutex>

#include <boost/format.hpp>

#include "utility/filedes.hpp"

#include "./tms-raster.hpp"

namespace generator {

class TmsWindyty : public TmsRaster {
public:
    TmsWindyty(const Params &params);

    struct DatasetConfig {
        std::time_t base;
        int period;
        std::string srs;
        math::Extents2 extents;
        std::string urlTemplate;
        int maxLod;
        int overviews;
        bool transparent;

        DatasetConfig()
            : base(), period(), maxLod(), overviews()
            , transparent(true)
        {}
    };

    struct File {
        std::time_t timestamp;
        std::string path;
        utility::Filedes fd;

        File() {}
        File(std::time_t timestamp, int pid, utility::Filedes &&fd)
            : timestamp(timestamp)
            , path(str(boost::format("/proc/%d/fd/%d") % pid % fd.get()))
            , fd(std::move(fd))
        {}
    };

private:
    virtual DatasetDesc dataset_impl() const;
    virtual bool transparent_impl() const;
    virtual bool hasMask_impl() const;

    struct Dataset {
        std::mutex mutex;

        File current;
        File prev;
    };

    int pid_;
    DatasetConfig dsConfig_;
    mutable Dataset ds_;
};

} // namespace generator

#endif // mapproxy_generator_tms_windyty_hpp_included_
