#include <ctime>
#include <algorithm>
#include <atomic>
#include <thread>
#include <memory>

#include <boost/noncopyable.hpp>
#include <boost/asio.hpp>
#include <boost/format.hpp>

#include <arpa/inet.h>

#include "dbglog/dbglog.hpp"

#include "utility/raise.hpp"
#include "utility/gccversion.hpp"

#include "./error.hpp"
#include "./http.hpp"

namespace asio = boost::asio;
namespace ip = asio::ip;

namespace  {

class Connection : boost::noncopyable {
public:
    typedef std::shared_ptr<Connection> pointer;

    Connection(asio::io_service &ios)
        : ios_(ios), socket_(ios_)
    {}

    ip::tcp::socket& socket() {return  socket_; }

    void start();

private:
    void readLine();

    asio::io_service &ios_;
    ip::tcp::socket socket_;
    asio::streambuf data_;
};

} // namespace

class Http::Detail : boost::noncopyable {
public:
    Detail(const utility::TcpEndpoint &listen, unsigned int threadCount
           , ContentGenerator &contentGenerator);
    ~Detail() { stop(); }

private:
    void start(std::size_t count);
    void stop();
    void worker(std::size_t id);

    void startAccept();

    ContentGenerator &contentGenerator_;

    asio::io_service ios_;
    asio::io_service::work work_;
    ip::tcp::acceptor acceptor_;
    std::vector<std::thread> workers_;
};

Http::Detail::Detail(const utility::TcpEndpoint &listen
                     , unsigned int threadCount
                     , ContentGenerator &contentGenerator)
    : contentGenerator_(contentGenerator)
    , work_(ios_)
    , acceptor_(ios_, listen.value, true)
{
    start(threadCount);
}

void Http::Detail::start(std::size_t count)
{
    // make sure threads are released when something goes wrong
    struct Guard {
        Guard(const std::function<void()> &func) : func(func) {}
        ~Guard() { if (func) { func(); } }
        void release() { func = {}; }
        std::function<void()> func;
    } guard(std::bind(&Detail::stop, this));

    for (std::size_t id(1); id <= count; ++id) {
        workers_.emplace_back(&Detail::worker, this, id);
    }

    guard.release();

    startAccept();
}

void Http::Detail::stop()
{
    ios_.stop();

    while (!workers_.empty()) {
        workers_.back().join();
        workers_.pop_back();
    }
}

void Http::Detail::worker(std::size_t id)
{
    dbglog::thread_id(str(boost::format("http:%u") % id));
    LOG(info2) << "Spawned HTTP worker id:" << id << ".";

    for (;;) {
        try {
            ios_.run();
            LOG(info2) << "Terminated HTTP worker id:" << id << ".";
            return;
        } catch (const std::exception &e) {
            LOG(err3)
                << "Uncaught exception in HTTP worker: <" << e.what()
                << ">. Going on.";
        }
    }
}

void Http::Detail::startAccept()
{
    auto conn(std::make_shared<Connection>(ios_));

    acceptor_.async_accept(conn->socket()
                           , [=](const boost::system::error_code &ec)
    {
        if (!ec) {
            conn->start();
        }

        startAccept();
    });
}

void Connection::start()
{
    LOG(info4) << "Accepted new connection.";
    readLine();
}

void Connection::readLine()
{
    asio::async_read_until(socket_, data_, "\r\n"
                           , [=](const boost::system::error_code &ec
                                 , std::size_t bytes)
    {
        if (ec) {
            // handle error
            return;
        }

        (void) bytes;

        {
            std::string method, uri, version;
            char sp1, sp2, cr, lf;
            std::istream is(&data_);
            is.unsetf(std::ios_base::skipws);
            is >> method >> sp1 >> uri >> sp2 >> version >> cr >> lf;
            LOG(info4) << "method: " << method;
            LOG(info4) << "uri: " << uri;
            LOG(info4) << "verion: " << version;
        }
    });
}

namespace {

const std::string error404(R"RAW(<html>
<head><title>404 Not Found</title></head>
<body bgcolor="white">
<center><h1>404 Not Found</h1></center>
)RAW");

const std::string error500(R"RAW(<html>
<head><title>500 Internal Server Error</title></head>
<body bgcolor="white">
<center><h1>500 Internal Server Error</h1></center>
)RAW");

const std::string error503(R"RAW(<html>
<head><title>503 Service Temporarily Unavailable</title></head>
<body bgcolor="white">
<center><h1>503 Service Temporarily Unavailable</h1></center>
)RAW");

const std::string error405(R"RAW(<html>
<head><title>405 Method Not Allowed</title></head>
<body bgcolor="white">
<center><h1>405 Method Not Allowed</h1></center>
)RAW");

const char *weekDays[7] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

std::string formatHttpDate(time_t time)
{
    if (time < 0) { time = std::time(nullptr); }
    tm bd;
    ::gmtime_r(&time, &bd);
    char buf[32];
    std::memcpy(buf, weekDays[bd.tm_wday], 3);
    std::strftime(buf + 3, sizeof(buf) - 1
                  , ", %d %b %Y %H:%M:%S GMT", &bd);
    return buf;
}

class HttpSink : public Sink {
public:
    HttpSink() {}
    ~HttpSink() {}

private:
    virtual void content_impl(const void *data, std::size_t size
                              , const FileInfo &stat, bool needCopy)
    {
        if (!valid()) { return; }
        (void) data;
        (void) size;
        (void) stat;
        (void) needCopy;

        // ri_.response.buffer(data, size, needCopy);
        // CHECK_MHD_ERROR(::MHD_add_response_header
        //                 (ri_.response.get(), MHD_HTTP_HEADER_CONTENT_TYPE
        //                  , stat.contentType.c_str())
        //                 , "Unable to set response header");
        // CHECK_MHD_ERROR(::MHD_add_response_header
        //                 (ri_.response.get(), MHD_HTTP_HEADER_LAST_MODIFIED
        //                  , formatHttpDate(stat.lastModified).c_str())
        //                  , "Unable to set response header");

        // enqueue();
    }

    virtual void seeOther_impl(const std::string &url)
    {
        if (!valid()) { return; }
        (void) url;

        // // TODO: compose body
        // ri_.response.buffer("", false);

        // ri_.responseCode = MHD_HTTP_SEE_OTHER;
        // ri_.location = url;

        // CHECK_MHD_ERROR(::MHD_add_response_header
        //                 (ri_.response.get(), MHD_HTTP_HEADER_LOCATION
        //                  , url.c_str())
        //                 , "Unable to set response header");
        // enqueue();
    }

    virtual void listing_impl(const Listing &list)
    {
        if (!valid()) { return; }

        std::string path;

        std::ostringstream os;
        os << R"RAW(<html>
<head><title>Index of )RAW" << path
           << R"RAW(</title></head>
<body bgcolor="white">
<h1>Index of )RAW"
           << path
           << "\n</h1><hr><pre><a href=\"../\">../</a>\n";

        auto sorted(list);
        std::sort(sorted.begin(), sorted.end());

        for (const auto &item : sorted) {
            switch (item.type) {
            case ListingItem::Type::file:
                os << "<a href=\"" << item.name << "\">"
                   << item.name << "</a>\n";
                break;
            case ListingItem::Type::dir:
                os << "<a href=\"" << item.name << "/\">"
                   << item.name << "/</a>\n";
                break;
            }
        }

        os << R"RAW(</pre><hr></body>
</html>
)RAW";

        content(os.str(), { "text/html" });
    }

    virtual void error_impl(const std::exception_ptr &exc)
    {
        if (!valid()) { return; }
        (void) exc;

        // const std::string *body;
        // try {
        //     std::rethrow_exception(exc);
        // } catch (const NotFound &e) {
        //     ri_.responseCode = MHD_HTTP_NOT_FOUND;
        //     ri_.errorReason = e.what();
        //     body = &error404;
        // } catch (const NotAllowed &e) {
        //     ri_.responseCode = MHD_HTTP_METHOD_NOT_ALLOWED;
        //     ri_.errorReason = e.what();
        //     body = &error405;
        // } catch (const Unavailable &e) {
        //     ri_.responseCode = MHD_HTTP_SERVICE_UNAVAILABLE;
        //     ri_.errorReason = e.what();
        //     body = &error503;
        // } catch (const std::exception &e) {
        //     ri_.responseCode = MHD_HTTP_INTERNAL_SERVER_ERROR;
        //     ri_.errorReason = e.what();
        //     body = &error500;
        // } catch (...) {
        //     ri_.responseCode = MHD_HTTP_INTERNAL_SERVER_ERROR;
        //     ri_.errorReason = "Unknown exception caught.";
        //     body = &error500;
        // }

        // LOG(debug) << "About to send http error: <" << ri_.errorReason << ">.";

        // // enqueue response with proper status and body; body is static string
        // // and thus not copied
        // ri_.response.buffer(*body, false);

        // CHECK_MHD_ERROR(::MHD_add_response_header
        //                 (ri_.response.get(), MHD_HTTP_HEADER_CONTENT_TYPE
        //                  , "text/html")
        //                 , "Unable to set response header");
        // enqueue();
    }

    bool finished() const {
        return false;
        // return (ri_.state == RequestInfo::State::finished);
    }

    bool checkAborted_impl() const {
        return finished();
    }

    bool valid() const {
        if (finished()) { return false; }

        // if (ri_.state != RequestInfo::State::handling) {
        //     LOG(warn3)
        //         << "Logic error in your code: attempt to send "
        //         "another response while it was already sent.";
        //     return false;
        // }
        return true;
    }

    void enqueue() {
        // ri_.enqueue(conn_);
    }
};

} // namespace

Http::Http(const utility::TcpEndpoint &listen, unsigned int threadCount
           , ContentGenerator &contentGenerator)
    : detail_(std::make_shared<Detail>(listen, threadCount, contentGenerator))
{
}
