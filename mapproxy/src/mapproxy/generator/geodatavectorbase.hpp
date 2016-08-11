#ifndef mapproxy_generator_geodata_hpp_included_
#define mapproxy_generator_geodata_hpp_included_

#include <boost/optional.hpp>

#include "geo/vectorformat.hpp"

#include "../generator.hpp"

namespace generator {

class GeodataVectorBase : public Generator {
public:
    GeodataVectorBase(const Params &params, bool tiled);

    struct Definition : public DefinitionBase {
        /** Input dataset (can be remote url, interpreted as a template by tiled
         *  version.
         */
        std::string dataset;
        std::string demDataset;
        boost::optional<std::string> geoidGrid;
        boost::optional<std::vector<std::string>> layers;
        geo::VectorFormat format;
        std::string styleUrl;
        int displaySize;

        /** Introspection surface.
         */
        boost::optional<Resource::Id> introspectionSurface;

        Definition()
            : format(geo::VectorFormat::geodataJson) , displaySize(256) {}
        bool operator==(const Definition &o) const;

        virtual void from_impl(const boost::any &value);
        virtual void to_impl(boost::any &value) const;

    private:
        virtual bool same_impl(const DefinitionBase &other) const {
            return (*this == other.as<Definition>());
        }
    };

private:
    virtual vr::FreeLayer freeLayer_impl(ResourceRoot root) const = 0;

    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , Sink &sink) const;

    virtual void generateMetatile(Sink &sink
                                  , const GeodataFileInfo &fileInfo
                                  , Arsenal &arsenal) const = 0;

    virtual void generateGeodata(Sink &sink
                                 , const GeodataFileInfo &fileInfo
                                 , Arsenal &arsenal) const = 0;

private:
    const Definition &definition_;
    bool tiled_;
};

} // namespace generator

#endif // mapproxy_generator_geodata_hpp_included_
