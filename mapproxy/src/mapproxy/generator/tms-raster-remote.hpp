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

#ifndef mapproxy_generator_tms_raster_remote_hpp_included_
#define mapproxy_generator_tms_raster_remote_hpp_included_

#include "../support/coverage.hpp"
#include "../generator.hpp"

namespace generator {

class TmsRasterRemote : public Generator {
public:
    TmsRasterRemote(const Params &params);

    struct Definition : public DefinitionBase {
        std::string remoteUrl;
        boost::optional<boost::filesystem::path> mask;

        Definition() {}

    private:
        virtual void from_impl(const boost::any &value);
        virtual void to_impl(boost::any &value) const;
        virtual Changed changed_impl(const DefinitionBase &other) const;
        virtual bool frozenCredits_impl() const { return false; }
    };

private:
    virtual void prepare_impl(Arsenal &arsenal);
    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const;

    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , Sink &sink) const;

    void generateTileMask(const vts::TileId &tileId
                          , const TmsFileInfo &fi
                          , Sink &sink
                          , Arsenal &arsenal) const;

    void generateTileMaskFromTree(const vts::TileId &tileId
                                  , const TmsFileInfo &fi
                                  , Sink &sink
                                  , Arsenal &arsenal) const;

    void generateMetatile(const vts::TileId &tileId
                          , const TmsFileInfo &fi
                          , Sink &sink
                          , Arsenal &arsenal) const;

    void generateMetatileFromTree(const vts::TileId &tileId
                                  , const TmsFileInfo &fi
                                  , Sink &sink
                                  , Arsenal &arsenal) const;

    vr::BoundLayer boundLayer(ResourceRoot root) const;

    const Definition &definition_;

    bool hasMetatiles_;

    // mask tree
    MaskTree maskTree_;

    /** Mask dataset path. Only when defined and not a RF tree.
     */
    boost::optional<std::string> maskDataset_;
};

} // namespace generator

#endif // mapproxy_generator_tms_raster_remote_hpp_included_
