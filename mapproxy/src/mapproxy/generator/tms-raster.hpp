#ifndef mapproxy_generator_tms_raster_hpp_included_
#define mapproxy_generator_tms_raster_hpp_included_

#include "vts-libs/vts/tileindex.hpp"

#include "../generator.hpp"

namespace generator {

class TmsRaster : public Generator {
public:
    TmsRaster(const Params &params);

    struct Definition : public DefinitionBase {
        std::string dataset;
        boost::optional<std::string> mask;
        RasterFormat format;
        bool transparent;

        Definition(): format(RasterFormat::jpg) {}

    private:
        virtual void from_impl(const boost::any &value);
        virtual void to_impl(boost::any &value) const;
        virtual Changed changed_impl(const DefinitionBase &other) const;
        virtual bool frozenCredits_impl() const { return false; }
    };

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

        DatasetDesc(const std::string &path
                    , const boost::optional<long> &maxAge = boost::none)
            : path(path), maxAge(maxAge)
        {}
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

    DatasetDesc dataset() const;

    RasterFormat format() const;

    bool transparent() const;

    bool hasMask() const;

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

    const Definition &definition_;

    bool hasMetatiles_;

    boost::optional<vts::TileIndex> index_;
};

inline TmsRaster::DatasetDesc TmsRaster::dataset() const {
    return dataset_impl();
}

inline bool TmsRaster::transparent() const {
    return transparent_impl();
}

inline bool TmsRaster::hasMask() const {
    return hasMask_impl();
}

} // namespace generator

#endif // mapproxy_generator_tms_raster_hpp_included_
