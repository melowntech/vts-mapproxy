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

#include <thread>

#include <boost/format.hpp>
#include <boost/asio.hpp>

#include "utility/raise.hpp"

#include "http/resourcefetcher.hpp"

#include "vts-libs/vts/mapconfig.hpp"
#include "vts-libs/registry.hpp"
#include "vts-libs/registry/extensions.hpp"

#include "fileinfo.hpp"
#include "error.hpp"
#include "core.hpp"
#include "sink.hpp"

namespace asio = boost::asio;
namespace vts = vtslibs::vts;
namespace vr = vtslibs::registry;
namespace vre = vtslibs::registry::extensions;

class Core::Detail : boost::noncopyable {
public:
    Detail(Generators &generators, GdalWarper &warper
           , unsigned int threadCount, http::ContentFetcher &contentFetcher)
        : resourceFetcher_(contentFetcher, &ios_)
        , generators_(generators)
        , arsenal_(warper, resourceFetcher_)
        , work_(ios_)
    {
        generators_.start(arsenal_);
        start(threadCount);
    }

    ~Detail() {
        stop();
        generators_.stop();
    }

    void generate(const http::Request &request, Sink sink);

    void generateResourceFile(const FileInfo &fi, Sink &sink);

    void generateListing(const FileInfo &fi, Sink &sink);

    void generateReferenceFrameDems(const FileInfo &fi, Sink &sink);

    bool assertBrowserEnabled(int flags, Sink &sink) const {
        if (flags & FileFlags::browserEnabled) { return true; }
        sink.error(utility::makeError<NotFound>("Browsing disabled."));
        return false;
    }

private:
    void start(std::size_t count);
    void stop();
    void worker(std::size_t id);
    void post(const Generator::Task &task, Sink sink);

    asio::io_service ios_;
    http::ResourceFetcher resourceFetcher_;

    Generators &generators_;
    Arsenal arsenal_;

    /** Processing pool stuff.
     */
    asio::io_service::work work_;
    std::vector<std::thread> workers_;
};

void Core::Detail::start(std::size_t count)
{
    // make sure threads are released when something goes wrong
    struct Guard {
        Guard(const std::function<void()> &func) : func(func) {}
        ~Guard() { if (func) { func(); } }
        void release() { func = {}; }
        std::function<void()> func;
    } guard([this]() { stop(); });

    for (std::size_t id(1); id <= count; ++id) {
        workers_.emplace_back(&Detail::worker, this, id);
    }

    guard.release();
}

void Core::Detail::stop()
{
    LOG(info2) << "Stopping core.";
    ios_.stop();

    while (!workers_.empty()) {
        workers_.back().join();
        workers_.pop_back();
    }
}

void Core::Detail::worker(std::size_t id)
{
    dbglog::thread_id(str(boost::format("core:%u") % id));
    LOG(info2) << "Spawned worker id:" << id << ".";

    for (;;) {
        try {
            ios_.run();
            LOG(info2) << "Terminated worker id:" << id << ".";
            return;
        } catch (const std::exception &e) {
            LOG(err3)
                << "Uncaught exception in worker: <" << e.what()
                << ">. Going on.";
        }
    }
}

void Core::Detail::post(const Generator::Task &task, Sink sink)
{
    if (!task) { return; }

    ios_.post([=]() mutable // sink is passed as non-const ref
    {
        try {
            task(sink, arsenal_);
        } catch (...) {
            sink.error();
        }
    });
}

Core::Core(Generators &generators, GdalWarper &warper
           , unsigned int threadCount, http::ContentFetcher &contentFetcher)
    : detail_(std::make_shared<Detail>
              (generators, warper, threadCount, contentFetcher))
{}

void Core::generate_impl(const http::Request &request
                         , const http::ServerSink::pointer &sink)
{
    detail().generate(request, Sink(sink));
}

namespace {

const std::string& getItem(const std::string &s)
{
    return s;
}

template <typename T>
const std::string& getName(const std::pair<std::string, T> &p)
{
    return p.first;
}

template <typename T>
const std::string& getName(const std::pair<const std::string, T> &p)
{
    return p.first;
}

template <typename T>
std::string getName(const T &value)
{
    return boost::lexical_cast<std::string>(value);
}

template <typename F, typename Container>
bool forEachEntry(const F &f, const Container &container)
{
    for (const auto &item : container) {
        f(getName(item));
    }
    return container.empty();
}

struct RfGeneratorTypes { std::string id; };

template <typename F>
bool forEachEntry(const F &f, const RfGeneratorTypes &rfgt)
{
    for (auto type : enumerationValues(Resource::Generator::Type())) {
        f(getName(type));
    }

    if (const vr::ReferenceFrame *rf
        = vr::system.referenceFrames(rfgt.id, std::nothrow))
    {
        if (rf->findExtension<vre::Tms>()) { f("terrain"); }
        if (rf->findExtension<vre::Wmts>()) { f("wmts"); }
    };

    return false;
}

template <bool allowEmpty, typename Container>
inline Sink::Listing
buildListing(const Container &container
             , const Sink::Listing &bootstrap = Sink::Listing())
{
    Sink::Listing out(bootstrap);

    auto empty(forEachEntry([&](const std::string &name)
    {
        out.emplace_back(name, Sink::ListingItem::Type::dir);
    }, container));

    if (!allowEmpty && empty) {
        throw NotFound("Empty listing");
    }

    return out;
}

template <typename Container>
inline Sink::Listing
buildListing(const Container &container
             , const Sink::Listing &bootstrap = Sink::Listing())
{
    return buildListing<true, Container>(container, bootstrap);
}

} // namespace

void Core::Detail::generate(const http::Request &request, Sink sink)
{
    try {
        FileInfo fi(request, generators_.config().fileFlags);

        switch (fi.type) {
        case FileInfo::Type::resourceFile:
            generateResourceFile(fi, sink);
            return;

        case FileInfo::Type::dirRedir:
            // append slash to filename and let browser try luck again
            sink.redirect(fi.filename + "/", utility::HttpCode::Found);
            return;

        case FileInfo::Type::referenceFrameListing:
        case FileInfo::Type::typeListing:
        case FileInfo::Type::groupListing:
        case FileInfo::Type::idListing:
            generateListing(fi, sink);
            return;

        case FileInfo::Type::referenceFrameDems:
            generateReferenceFrameDems(fi, sink);
            return;

        default: break;
        }

        sink.error(utility::makeError<InternalError>("Unhandled."));
    } catch (...) {
        sink.error();
    }
}

void Core::Detail::generateResourceFile(const FileInfo &fi, Sink &sink)
{
    auto generator(generators_.generator(fi));
    if (!generator) {
        sink.error(utility::makeError<NotFound>
                    ("No generator for URL <%s> found.", fi.url));
        return;
    }

    // assign file class stuff
    sink.assignFileClassSettings(generator->resource().fileClassSettings);

    // run machinery
    post(generator->generateFile(fi, sink), sink);
}

void Core::Detail::generateReferenceFrameDems(const FileInfo &fi, Sink &sink)
{
    if (!assertBrowserEnabled(fi.flags, sink)) { return; }

    const auto &referenceFrame(fi.resourceId.referenceFrame);

    auto records(generators_.demRegistry().records(referenceFrame));

    std::ostringstream os;
    os << R"RAW(<html>
<head><title>DEM mapping for )RAW" << referenceFrame
       << R"RAW(</title></head>
<body bgcolor="white">
<h1>DEM mapping for )RAW"
       << referenceFrame
       << "\n</h1><hr><pre><a href=\"../\">../</a>\n";

    for (const auto &record : records) {
        auto link(prependRoot(boost::filesystem::path()
                              , record.resourceId
                              , Resource::Generator::Type::surface
                              , ResourceRoot::Depth::interface));

        os << "<a href=\"" << link.string() << "\">"
                   << record.id.id << "</a>\n";
    }

    os << R"RAW(</pre><hr></body>
</html>
)RAW";

    sink.content(os.str(), { "text/html; charset=utf-8", -1, -1 });
}

namespace {

Sink::Listing browsableDirectoryContent = {
#if 0
    { "index.html" }
    , { "mapConfig.json" }
#endif
};

Sink::Listing otherDirectoryContent = {
#if 0
    { "index.html" }
#endif
};

Sink::Listing rfDirectoryContent = {
#if 0
    { "index.html" },
    { "mapConfig.json" },
#endif
    { "dems.html" }
};

} // namespace

void Core::Detail::generateListing(const FileInfo &fi, Sink &sink)
{
    if (!assertBrowserEnabled(fi.flags, sink)) { return; }

    switch (fi.type) {
    case FileInfo::Type::referenceFrameListing:
        sink.listing(buildListing(vr::system.referenceFrames
                                   , otherDirectoryContent));
        return;

    case FileInfo::Type::typeListing:
        sink.listing
            (buildListing(RfGeneratorTypes{fi.resourceId.referenceFrame}
                          , rfDirectoryContent));
        return;

    case FileInfo::Type::groupListing:
        sink.listing(buildListing
                      (generators_.listGroups
                       (fi.resourceId.referenceFrame, fi.interface.type)
                       , browsableDirectoryContent));
        return;

    case FileInfo::Type::idListing:
        sink.listing(buildListing<false>
                      (generators_.listIds
                       (fi.resourceId.referenceFrame, fi.interface.type
                        , fi.resourceId.group)
                       , browsableDirectoryContent));
        return;

    default: break;
    }

    sink.error(utility::makeError<InternalError>("Unhandled."));
}
