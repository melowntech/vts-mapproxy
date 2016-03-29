#include "utility/raise.hpp"

#include "vts-libs/vts/mapconfig.hpp"

#include "./fileinfo.hpp"
#include "./error.hpp"
#include "./core.hpp"

namespace vts = vadstena::vts;

struct Core::Detail : boost::noncopyable {
    Detail(Generators &generators)
        : generators(generators)
    {}

    void generate(const std::string &location
                  , const Sink::pointer &sink);

    void generateRfMapConfig(const std::string &referenceFrame
                             , const Sink::pointer &sink);

    void generateResourceFile(const FileInfo &fi
                              , const Sink::pointer &sink);

    Generators &generators;
};

Core::Core(Generators &generators)
    : detail_(std::make_shared<Detail>(generators))
{}

void Core::generate_impl(const std::string &location
                         , const Sink::pointer &sink)
{
    detail().generate(location, sink);
}

void Core::Detail::generate(const std::string &location
                            , const Sink::pointer &sink)
{
    try {
        FileInfo fi(location);

        switch (fi.type) {
        case FileInfo::Type::rfMapConfig:
            generateRfMapConfig(fi.referenceFrame, sink);
            break;

        case FileInfo::Type::resourceFile:
            generateResourceFile(fi, sink);
            break;
        }
    } catch (...) {
        sink->error();
    }
}

void Core::Detail::generateRfMapConfig(const std::string &referenceFrame
                                       , const Sink::pointer &sink)
{
    auto genlist(generators.referenceFrame(referenceFrame));
    if (genlist.empty()) {
        sink->error(utility::makeError<NotFound>
                    ("No data for <%s>.", referenceFrame));
    }

    // build map
    vts::MapConfig mapConfig;
    for (const auto &generator : genlist) {
        mapConfig.merge(generator->mapConfig
                        (referenceFrame, ResourceRoot::type));
    }

    std::ostringstream os;
    vts::saveMapConfig(mapConfig, os);
    sink->content(os.str(), Sink::FileInfo("application/json"));
}

void Core::Detail::generateResourceFile(const FileInfo &fi
                                        , const Sink::pointer &sink)
{
    auto generator(generators.generator(fi));
    (void) generator;
    (void) sink;
}
