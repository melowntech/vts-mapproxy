#ifndef mapproxy_generator_tms_raster_hpp_included_
#define mapproxy_generator_tms_raster_hpp_included_

#include "../generator.hpp"

namespace generator {

class TmsRaster : public Generator {
public:
    TmsRaster(const Params &params);

    struct Definition : public DefinitionBase {
        std::string dataset;
        boost::optional<std::string> mask;
        RasterFormat format;

        Definition(): format(RasterFormat::jpg) {}
        bool operator==(const Definition &o) const;

    private:
        virtual void from_impl(const boost::any &value);
        virtual void to_impl(boost::any &value) const;
        virtual bool same_impl(const DefinitionBase &other) const {
            return (*this == other.as<Definition>());
        }
    };

private:
    virtual void prepare_impl();
    virtual vts::MapConfig mapConfig_impl(ResourceRoot root) const;

    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , Sink &sink) const;

    void generateTileImage(const vts::TileId &tileId
                           , Sink &sink, Arsenal &arsenal) const;

    void generateTileMask(const vts::TileId &tileId
                          , Sink &sink, Arsenal &arsenal) const;

    void generateMetatile(const vts::TileId &tileId
                          , Sink &sink, Arsenal &arsenal) const;

    vr::BoundLayer boundLayer(ResourceRoot root) const;

    const Definition &definition_;

    bool hasMetatiles_;
};

} // namespace generator

#endif // mapproxy_generator_tms_raster_hpp_included_
