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

#ifndef mapproxy_generator_tms_raster_hpp_included_
#define mapproxy_generator_tms_raster_hpp_included_

#include "geo/geodataset.hpp"

#include "../support/coverage.hpp"
#include "../support/mmapped/tileindex.hpp"

#include "../definition/tms.hpp"

#include "tms-raster-base.hpp"

namespace generator {

namespace detail {

/** Member from base idiom
 */
class TmsRasterMFB {
protected:
    TmsRasterMFB(const Generator::Params &params);
    typedef resource::TmsRaster Definition;
    const Definition &definition_;
};

} // namespace detail

class TmsRaster
    : private detail::TmsRasterMFB
    , public TmsRasterBase
{
public:
    TmsRaster(const Params &params
              , const boost::optional<RasterFormat> &format = boost::none);

    using detail::TmsRasterMFB::Definition;

protected:
    const Definition &definition() const { return definition_; }

    /** Dataset descriptor.
     */
    struct DatasetDesc {
        /** Path to dataset.
         */
        std::string path;

        /** Maximum allowed age of this dataset. Should be translated into
         *  Cache-Control max-age is valid.
         */
        boost::optional<long> maxAge;

        /** Is dataset dynamic? No shortcuts are performed in the case of
         *  dynamic processing.
         */
        bool dynamic;

        DatasetDesc(const std::string &path
                    , const boost::optional<long> &maxAge = boost::none
                    , bool dynamic = false)
            : path(path), maxAge(maxAge), dynamic(dynamic)
        {}
    };

protected:
    virtual vr::BoundLayer boundLayer(ResourceRoot root) const;

private:
    virtual void prepare_impl(Arsenal &arsenal);
    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const;

    virtual Task generateVtsFile_impl(const FileInfo &fileInfo
                                      , Sink &sink) const;

    virtual void generateTileImage(const vts::TileId &tileId
                                   , const Sink::FileInfo &fi
                                   , RasterFormat format
                                   , Sink &sink, Arsenal &arsenal
                                   , const ImageFlags &imageFlags
                                   = ImageFlags()) const;

    void generateTileMask(const vts::TileId &tileId
                          , const TmsFileInfo &fi
                          , Sink &sink, Arsenal &arsenal) const;

    void generateTileMaskFromTree(const vts::TileId &tileId
                                  , const TmsFileInfo &fi
                                  , Sink &sink
                                  , Arsenal&) const;

    void generateMetatile(const vts::TileId &tileId
                          , const TmsFileInfo &fi
                          , Sink &sink, Arsenal &arsenal) const;

    DatasetDesc dataset() const;

    RasterFormat format() const;

    bool transparent() const;

    bool hasMask() const;

    void update(vr::BoundLayer &bl) const;

    // customizable stuff

    /** Path to dataset and its validity. Defaults to path from resource.
     */
    virtual DatasetDesc dataset_impl() const;

    /** Reports bound layer as a transparent one. Forces raster format to PNG>
     */
    virtual bool transparent_impl() const;

    /** Advertise mask definition to the user?
     */
    virtual bool hasMask_impl() const;

    /** Update boundlayer information before sending to client. Defaults to
     *  no-op.
     */
    virtual void update_impl(vr::BoundLayer &bl) const;

    bool hasMetatiles_;

    boost::optional<mmapped::TileIndex> index_;

    bool complexDataset_;

    // mask tree
    MaskTree maskTree_;

    /** Mask dataset path. Only when defined and not a RF tree.
     */
    boost::optional<std::string> maskDataset_;
};

// inlines

inline TmsRaster::DatasetDesc TmsRaster::dataset() const {
    return dataset_impl();
}

inline bool TmsRaster::transparent() const {
    return transparent_impl();
}

inline bool TmsRaster::hasMask() const {
    return hasMask_impl();
}

inline void TmsRaster::update(vr::BoundLayer &bl) const {
    return update_impl(bl);
}

inline void TmsRaster::update_impl(vr::BoundLayer&) const {}

} // namespace generator

#endif // mapproxy_generator_tms_raster_hpp_included_
