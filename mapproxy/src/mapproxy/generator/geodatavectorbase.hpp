#ifndef mapproxy_generator_geodata_hpp_included_
#define mapproxy_generator_geodata_hpp_included_

#include "geo/vectorformat.hpp"

#include "../generator.hpp"

namespace generator {

class GeodataVectorBase : public Generator {
public:
    GeodataVectorBase(const Config &config, const Resource &resource
                      , bool tiled);

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

        Definition(): format(geo::VectorFormat::geodataJson) {}
        bool operator==(const Definition &o) const;

    private:
        virtual void from_impl(const boost::any &value);
        virtual void to_impl(boost::any &value) const;
        virtual bool same_impl(const DefinitionBase &other) const {
            return (*this == other.as<Definition>());
        }
    };

private:
    virtual Task generateFile_impl(const FileInfo &fileInfo
                                   , Sink &sink) const;

    virtual void generateMetatile(Sink &sink
                                  , const GeodataFileInfo &fileInfo
                                  , Arsenal &arsenal) const = 0;

    virtual void generateGeodata(Sink &sink
                                 , const GeodataFileInfo &fileInfo
                                 , Arsenal &arsenal) const = 0;

protected:
    const Definition &definition_;

private:
    bool tiled_;
};

} // namespace generator

#endif // mapproxy_generator_geodata_hpp_included_
