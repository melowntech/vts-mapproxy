#include <algorithm>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include "dbglog/dbglog.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/storage/fstreams.hpp"

#include "../support/python.hpp"

#include "./geodatavectorbase.hpp"

namespace generator {

namespace {

void parseDefinition(GeodataVectorBase::Definition &def
                     , const Json::Value &value)
{
    std::string s;

    Json::get(def.dataset, value, "dataset");
    Json::get(def.demDataset, value, "demDataset");

    if (value.isMember("geoidGrid")) {
        def.geoidGrid = boost::in_place();
        Json::get(*def.geoidGrid, value, "geoidGrid");
    }

    if (value.isMember("layers")) {
        const auto layers(value["layers"]);
        if (!layers.isArray()) {
            LOGTHROW(err1, InternalError)
                << "Cannot find imageUrl in bind metadata reply.";
        }

        def.layers = boost::in_place();
        for (const auto &layer : layers) {
            def.layers->push_back(layer.asString());
        }
        std::sort(def.layers->begin(), def.layers->end());
    }

    if (value.isMember("format")) {
        Json::get(s, value, "format");
        try {
            def.format
                = boost::lexical_cast<geo::VectorFormat>(s);
        } catch (boost::bad_lexical_cast) {
            utility::raise<Error>
                ("Value stored in format is not a valid height"
                 " coded data format.");
        }
    }

    Json::get(def.styleUrl, value, "styleUrl");
}

void buildDefinition(Json::Value &value
                     , const GeodataVectorBase::Definition &def)
{
    value["dataset"] = def.dataset;
    value["demDataset"] = def.demDataset;

    if (def.geoidGrid) { value["geoidGrid"] = *def.geoidGrid;  }

    if (def.layers) {
        auto &layers(value["layers"] = Json::arrayValue);
        for (const auto &layer : *def.layers) {
            layers.append(layer);
        }
    }

    value["format"] = boost::lexical_cast<std::string>(def.format);
    value["styleUrl"] = def.styleUrl;
}

void parseDefinition(GeodataVectorBase::Definition &def
                     , const boost::python::dict &value)
{
    def.dataset = py2utf8(value["dataset"]);
    def.demDataset = py2utf8(value["demDataset"]);

    if (value.has_key("geoidGrid")) {
        def.geoidGrid = py2utf8(value["mask"]);
    }

    if (value.has_key("layers")) {
        def.layers = boost::in_place();
        auto pylayers(value["layers"]);
        for (boost::python::stl_input_iterator<boost::python::object>
                 ipylayers(pylayers), epylayers(pylayers);
             ipylayers != epylayers; ++ipylayers)
        {
            def.layers->push_back(py2utf8(*ipylayers));
        }
        std::sort(def.layers->begin(), def.layers->end());
    }

    if (value.has_key("format")) {
        try {
            def.format = boost::lexical_cast<geo::VectorFormat>
                (py2utf8(value["format"]));
        } catch (boost::bad_lexical_cast) {
            utility::raise<Error>
                ("Value stored in format is not a valid height"
                 " coded data format.");
        }
    }

    def.styleUrl = py2utf8(value["styleUrl"]);
}

} // namespace

void GeodataVectorBase::Definition::from_impl(const boost::any &value)
{
    if (const auto *json = boost::any_cast<Json::Value>(&value)) {
        parseDefinition(*this, *json);
    } else if (const auto *py
               = boost::any_cast<boost::python::dict>(&value))
    {
        parseDefinition(*this, *py);
    } else {
        LOGTHROW(err1, Error)
            << "GeodataVectorBase: Unsupported configuration from: <"
            << value.type().name() << ">.";
    }
}

void GeodataVectorBase::Definition::to_impl(boost::any &value) const
{
    if (auto *json = boost::any_cast<Json::Value>(&value)) {
        buildDefinition(*json, *this);
    } else {
        LOGTHROW(err1, Error)
            << "GeodataVectorBase:: Unsupported serialization into: <"
            << value.type().name() << ">.";
    }
}

bool GeodataVectorBase::Definition::operator==(const Definition &o) const
{
    if (dataset != o.dataset) { return false; }
    if (demDataset != o.demDataset) { return false; }
    if (geoidGrid != o.geoidGrid) { return false; }
    if (layers != o.layers) { return false; }

    // format can change
    return true;
}

GeodataVectorBase::GeodataVectorBase(const Config &config
                                     , const Resource &resource
                                     , bool tiled)
    : Generator(config, resource)
    , definition_(this->resource().definition<Definition>())
    , tiled_(tiled)
{}

Generator::Task GeodataVectorBase::generateFile_impl(const FileInfo &fileInfo
                                                     , Sink &sink) const
{
    GeodataFileInfo fi(fileInfo, tiled_, definition_.format);

    // check for valid tileId
    switch (fi.type) {
    case GeodataFileInfo::Type::geo:
        return[=](Sink &sink, Arsenal &arsenal) {
            generateGeodata(sink, fi, arsenal);
        };
        break;

    case GeodataFileInfo::Type::metatile:
        if (tiled_) {
            return[=](Sink &sink, Arsenal &arsenal) {
                generateMetatile(sink, fi, arsenal);
            };
        }
        sink.error(utility::makeError<NotFound>
                   ("Metatiles not supported by non-tiled driver."));
        break;

    case GeodataFileInfo::Type::config: {
        std::ostringstream os;
        mapConfig(os, ResourceRoot::none);
        sink.content(os.str(), fi.sinkFileInfo());
        break;
    }

    case GeodataFileInfo::Type::support:
        sink.content(fi.support->data, fi.support->size
                      , fi.sinkFileInfo(), false);
        break;

    case GeodataFileInfo::Type::registry:
        sink.content(vs::fileIStream
                      (fi.registry->contentType, fi.registry->path));
        break;

    default:
        sink.error(utility::makeError<InternalError>
                    ("Not implemented yet."));
        break;
    }

    return {};
}

} // namespace generator
