/**
 * Copyright (c) 2017 Melown Technologies SE
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

#ifndef mapproxy_generator_tms_windyty_hpp_included_
#define mapproxy_generator_tms_windyty_hpp_included_

#include <mutex>

#include <boost/format.hpp>

#include "utility/filedes.hpp"

#include "tms-raster.hpp"

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
        int forecastOffset;

        DatasetConfig()
            : base(), period(), maxLod(), overviews()
            , transparent(true)
        {}
    };

    struct File {
        std::time_t timestamp;
        std::string path;

        File() = default;
        File(const File&) = delete;
        File& operator=(const File&) = delete;

        File(std::time_t timestamp, const std::string &path)
            : timestamp(timestamp), path(path)
        {}

        File(File &&o)
            : timestamp(o.timestamp), path(o.path)
        {
            o.path.clear();
        }

        File& operator=(File &&o) {
            if (this == &o) { return *this; }

            if (path == o.path) {
                // same path -> do not remove and unsed from the other one
                o.path.clear();
            } else {
                // different path, remove this one
                remove();
                std::swap(path, o.path);
            }

            timestamp = o.timestamp;
            return *this;
        }

        ~File() { remove(); }

    private:
        void remove();
    };

    typedef resource::TmsWindyty Definition;

private:
    virtual DatasetDesc dataset_impl() const;
    virtual bool transparent_impl() const;
    virtual bool hasMask_impl() const;

    virtual vr::BoundLayer boundLayer(ResourceRoot root) const;

    struct DsInfo {
        std::string path;
        std::time_t timestamp;
    };

    DsInfo dsInfo(std::time_t now) const;

    struct Dataset {
        std::mutex mutex;

        File current;
        File prev;
    };

    const Definition &definition_;
    int pid_;
    DatasetConfig dsConfig_;
    mutable Dataset ds_;
};

} // namespace generator

#endif // mapproxy_generator_tms_windyty_hpp_included_
