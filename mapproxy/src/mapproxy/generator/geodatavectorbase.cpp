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

#include <algorithm>

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/uri.hpp"

#include "jsoncpp/json.hpp"
#include "jsoncpp/as.hpp"

#include "vts-libs/storage/fstreams.hpp"

#include "../support/python.hpp"

#include "geodatavectorbase.hpp"
#include "files.hpp"

namespace ba = boost::algorithm;
namespace vf = geo::vectorformat;

namespace generator {

GeodataVectorBase::GeodataVectorBase(const Params &params, bool tiled)
    : Generator(params)
    , definition_(this->resource().definition<Definition>())
    , layerEnhancers_(definition_.layerEnhancers)
    , tiled_(tiled)
    , styleUrl_(definition_.styleUrl)
{
    if (styleUrl_.empty()) {
        styleUrl_ = "style.json";
    } else if (ba::istarts_with(styleUrl_, "file:")) {
        // pseudo file URL
        stylePath_ = absoluteDataset(styleUrl_.substr(5));
        styleUrl_ = "style.json";
    }

    for (auto &item : layerEnhancers_) {
        item.second.databasePath = absoluteDataset(item.second.databasePath);
    }
}

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

    case GeodataFileInfo::Type::definition: {
        std::ostringstream os;
        vr::saveFreeLayer(os, freeLayer_impl(ResourceRoot::none));
        sink.content(os.str(), fi.sinkFileInfo());
        break;
    }

    case GeodataFileInfo::Type::support:
        supportFile(*fi.support, sink, fi.sinkFileInfo());
        break;

    case GeodataFileInfo::Type::registry:
        sink.content(vs::fileIStream
                      (fi.registry->contentType, fi.registry->path)
                     , FileClass::registry);
        break;

    case GeodataFileInfo::Type::style:
        if (stylePath_.empty()) {
            // return internal file
            supportFile(files::defaultStyle, sink, fi.sinkFileInfo());
        } else {
            // return external file
            sink.content(vs::fileIStream
                         (files::defaultStyle.contentType, stylePath_)
                         , FileClass::config);
        }
        break;

    default:
        sink.error(utility::makeError<NotFound>("Not Found."));
        break;
    }

    return {};
}

namespace {

typedef boost::iterator_range<std::string::const_iterator> SubString;
typedef std::vector<SubString> Args;
typedef std::pair<SubString, SubString> KeyValue;

KeyValue splitArgument(const SubString &arg)
{
    auto b(std::begin(arg));
    auto e(std::end(arg));
    for (auto i(b); i != e; ++i) {
        if (*i == '=') {
            return KeyValue(SubString(b, i), SubString(std::next(i), e));
        }
    }
    return KeyValue(SubString(b, e), SubString());
}

} // namespace

std::pair<DemDataset::list, bool>
GeodataVectorBase::viewspec2datasets(const std::string &query
                                     , const DemDataset &fallback)
    const
{
    auto empty([&]() -> std::pair<DemDataset::list, bool>
    {
        return { DemDataset::list{fallback}, true };
    });

    if (query.empty()) {
        LOG(info1) << "No query -. no viewspec.";
        return empty();
    };

    Args args;
    ba::split(args, query, ba::is_any_of("&"), ba::token_compress_on);

    for (auto iargs(args.begin()), eargs(args.end()); iargs != eargs; ++iargs)
    {
        auto kv(splitArgument(*iargs));
        if (ba::equals(kv.first, "viewspec")) {
            std::vector<std::string> ids;
            ba::split(ids, kv.second, ba::is_any_of(",")
                      , ba::token_compress_on);

            // url decode values
            for (auto &id : ids) { id = utility::urlDecode(id); }

            if ((ids.size() == 1)
                && ((ids.front() == "{viewspec}")
                    || (ids.front() == "viewspec")))
            {
                // viewspec not expanded, treate as empty
                LOG(info1) << "Viewspec not expanded, ignoring.";
                return empty();
            }

            auto result(demRegistry().find(referenceFrameId(), ids));
            result.first.emplace_back(fallback);
            return result;
        }
    }

    // nothing appropriate
    LOG(info1) << "No query -> no viewspec.";
    return empty();
}

} // namespace generator
