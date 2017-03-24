#ifndef mapproxy_generator_tms_raster_patchwork_hpp_included_
#define mapproxy_generator_tms_raster_patchwork_hpp_included_

#include "vts-libs/vts/tileindex.hpp"

#include "../generator.hpp"

namespace generator {

class TmsRasterPatchwork : public Generator {
public:
    TmsRasterPatchwork(const Params &params);

    struct Definition : public DefinitionBase {
        boost::optional<std::string> mask;
        RasterFormat format;

        Definition(): format(RasterFormat::jpg) {}

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

    void generateTileImage(const vts::TileId &tileId
                           , const TmsFileInfo &fi
                           , Sink &sink, Arsenal &arsenal) const;

    void generateTileMask(const vts::TileId &tileId
                          , const TmsFileInfo &fi
                          , Sink &sink, Arsenal &arsenal) const;

    void generateMetatile(const vts::TileId &tileId
                          , const TmsFileInfo &fi
                          , Sink &sink, Arsenal &arsenal) const;

    vr::BoundLayer boundLayer(ResourceRoot root) const;

    bool hasMask() const;

    const Definition &definition_;

    bool hasMetatiles_;
};

} // namespace generator

#endif // mapproxy_generator_tms_raster_patchwork_hpp_included_
