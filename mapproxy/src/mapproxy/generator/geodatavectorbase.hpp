#ifndef mapproxy_generator_geodata_hpp_included_
#define mapproxy_generator_geodata_hpp_included_

#include "geo/heightcoding.hpp"

#include "../generator.hpp"

namespace generator {

class GeodataVectorBase : public Generator {
public:
    GeodataVectorBase(const Config &config, const Resource &resource);

    struct Definition : public DefinitionBase {
        /** Input dataset (can be remote url, interpreted as a template by tiled
         *  version.
         */
        std::string dataset;
        std::string demDataset;
        boost::optional<std::string> geoidGrid;
        boost::optional<geo::HeightCodingConfig::LayerNames> layers;
        geo::HeightCodingConfig::Format format;
        std::string styleUrl;

        Definition(): format(geo::HeightCodingConfig::Format::geodataJson) {}
        bool operator==(const Definition &o) const;

    private:
        virtual void from_impl(const boost::any &value);
        virtual void to_impl(boost::any &value) const;
        virtual bool same_impl(const DefinitionBase &other) const {
            return (*this == other.as<Definition>());
        }
    };

protected:
    const Definition &definition_;
};

} // namespace generator

#endif // mapproxy_generator_geodata_hpp_included_
