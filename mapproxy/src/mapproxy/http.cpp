#include <ctime>
#include <algorithm>
#include <atomic>

#include <boost/noncopyable.hpp>

#include <arpa/inet.h>

#include <microhttpd.h>

#include "dbglog/dbglog.hpp"

#include "utility/raise.hpp"
#include "utility/gccversion.hpp"

#include "./error.hpp"
#include "./http.hpp"

namespace {

UTILITY_THREAD_LOCAL std::string *lastError = nullptr;

UTILITY_THREAD_LOCAL std::size_t thisIsHttpThread = 0;

std::atomic<std::size_t> httpThreadIdGenerator(0);

void setLastError(const char *value)
{
    if (lastError) {
        LOG(warn2) << "Last microhttpd library error in this thread has "
            "not been reclaimed. It read: <" << *lastError << ">.";
        delete lastError;
    }
    lastError = new std::string(value);
}

std::string getLastError()
{
    std::string value("unknown");
    if (lastError) {
        value = *lastError;
        delete lastError;
    }
    return value;
}

#define CHECK_MHD_ERROR(res, what)                              \
    if (!res) {                                                 \
        LOGTHROW(err2, IOError)                                 \
            << what << "; reason: <" << getLastError() << ">."; \
    }

struct ClientInfo {
    const ::sockaddr *addr;

    ClientInfo(::MHD_Connection *connection)
        : addr()
    {
        auto client(::MHD_get_connection_info
                (connection, MHD_CONNECTION_INFO_CLIENT_ADDRESS));
        if (client) { addr = client->client_addr; }
    }
};

class ResponseHandle : boost::noncopyable {
public:
    ResponseHandle() : response_() {}

    ~ResponseHandle() {
        if (response_) { ::MHD_destroy_response(response_); }
    }

    void buffer(const std::string &data, bool copy) {
        buffer(data.data(), data.size(), copy);
    }

    void buffer(const void *data, std::size_t size, bool copy) {
        response_ = ::MHD_create_response_from_buffer
            (size, const_cast<void*>(data)
             , copy ? MHD_RESPMEM_MUST_COPY : MHD_RESPMEM_PERSISTENT);
    }

    MHD_Response* get() { return response_; }
    operator MHD_Response*() { return get(); }
    operator bool() const { return response_; }

private:
    MHD_Response *response_;
};

struct RequestInfo : boost::noncopyable {
    struct State {
        enum {
            fresh, handling, enqueueable, enqueued, finished
        };
    };
    std::atomic<int> state;

    std::string url;
    std::string method;
    std::string version;
    int responseCode;
    std::string errorReason;
    std::string location;

    ResponseHandle response;

    RequestInfo(const std::string &url, const std::string &method
                , const std::string &version)
        : state(State::fresh)
        , url(url), method(method), version(version)
        , responseCode(MHD_HTTP_OK)
        , refcount_(1)
    {}

    void enqueue(::MHD_Connection *connection) {
        if (thisIsHttpThread) {
            // marked as HTTP thread, just enqueue
            LOG(info2) << "Enqueueing response.";
            CHECK_MHD_ERROR(::MHD_queue_response
                            (connection, responseCode, response)
                            , "Unable to enqueue HTTP response");
            state = State::enqueued;
        } else {
            // force event loop wakeup (suspend and immediately resume)
            // TODO: make better via another pipe
            state = State::enqueueable;
            ::MHD_suspend_connection(connection);
            ::MHD_resume_connection(connection);
        }
    }

    static RequestInfo *addRef(RequestInfo *ri) {
        ri->inc();
        return ri;
    };

    static void decRef(RequestInfo *ri) {
        if (ri->dec()) {
            delete ri;
        }
    };

private:
    /** Increment reference count.
     */
    void inc() { ++refcount_; }

    /** Decrement reference count and return true when it reaches zero.
     */
    bool dec() { return !--refcount_; }

    std::atomic<std::size_t> refcount_;
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const ClientInfo &ci)
{
    if (!ci.addr) { return os << '-'; }

    char buf[INET_ADDRSTRLEN + 1];
    const auto *addr(reinterpret_cast<const ::sockaddr_in*>(ci.addr));
    os << ::inet_ntop(AF_INET, &addr->sin_addr, buf, INET_ADDRSTRLEN + 1);

    return os;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const RequestInfo &ri)
{
    return os << '"' << ri.method << " " << ri.url << ' ' << ri.version
              << '"';
}

} // namespace

struct Http::Detail : boost::noncopyable {
    Detail(const utility::TcpEndpoint &listen, unsigned int threadCount
           , ContentGenerator &contentGenerator);
    ~Detail() {
        if (daemon) { ::MHD_stop_daemon(daemon); }
    }

    int request(::MHD_Connection *connection, RequestInfo *requestInfo);

    ContentGenerator &contentGenerator;
    ::MHD_Daemon *daemon;
};

extern "C" {

void mapproxy_http_callback_completed(void*, ::MHD_Connection *connection
                                      , void **info
                                      , enum MHD_RequestTerminationCode toe)
{
    // Request info holder/deleter
    class Deleter {
    public:
        Deleter(void **info)
            : info_(info), ri_(static_cast<RequestInfo*>(*info_))
        {}

        ~Deleter() {
            // decrement reference
            RequestInfo::decRef(ri_);
            // and get rid of pointer
            *info_ = nullptr;
        }

        RequestInfo& get() { return *ri_; }

    private:
        void **info_;
        RequestInfo *ri_;
    } deleter(info);

    auto &rinfo(deleter.get());
    // mark as finished
    rinfo.state = RequestInfo::State::finished;

    switch (toe) {
    case MHD_REQUEST_TERMINATED_COMPLETED_OK:
        switch (rinfo.responseCode / 100) {
        case 2:
        case 3:
            if (rinfo.location.empty()) {
                LOG(info3)
                    << "HTTP " << ClientInfo(connection) << ' ' << rinfo
                    << ' ' << rinfo.responseCode << ".";
            } else {
                LOG(info3)
                    << "HTTP " << ClientInfo(connection) << ' ' << rinfo
                    << ' ' << rinfo.responseCode
                    << " -> \"" << rinfo.location << "\".";
            }
            return;

        default:
            LOG(err2)
                << "HTTP " << ClientInfo(connection) << ' ' << rinfo
                << ' ' << rinfo.responseCode << "; reason: <"
                << rinfo.errorReason << ">.";
        }
        return;

    case MHD_REQUEST_TERMINATED_WITH_ERROR:
        LOG(err2)
             << "HTTP " << ClientInfo(connection) << ' ' << rinfo
             << " [internal error].";
        return;

    case MHD_REQUEST_TERMINATED_TIMEOUT_REACHED:
        LOG(err2)
            << "HTTP " << ClientInfo(connection) << ' ' << rinfo
            << " [timed-out].";
        return;

    case MHD_REQUEST_TERMINATED_DAEMON_SHUTDOWN:
        LOG(err2)
            << "HTTP " << ClientInfo(connection) << ' ' << rinfo
            << " [shutdown].";
        return;

    case MHD_REQUEST_TERMINATED_READ_ERROR:
        LOG(err2)
            << "HTTP " << ClientInfo(connection) << ' ' << rinfo
            << " [read error].";
        return;

    case MHD_REQUEST_TERMINATED_CLIENT_ABORT:
        LOG(err2)
            << "HTTP " << ClientInfo(connection) << ' ' << rinfo
            << " [aborted].";
        return;
    }
}

int mapproxy_http_callback_request(void *cls, ::MHD_Connection *connection
                                   , const char *url, const char *method
                                   , const char *version
                                   , const char*, size_t*, void **info)
{
    // mark as http thread
    if (!thisIsHttpThread) {
        thisIsHttpThread = ++httpThreadIdGenerator;
        dbglog::thread_id(str(boost::format("http:%u") % thisIsHttpThread));
    }

    if (!*info) {
        // setup, log and done
        auto rinfo(new RequestInfo(url, method, version));
        *info = rinfo;

        // received request logging
        LOG(info2)
            << "HTTP " << ClientInfo(connection)  << ' ' << *rinfo
            << ".";

        return MHD_YES;
    }

    // second call -> process
    auto *rinfo(static_cast<RequestInfo*>(*info));

    switch (rinfo->state) {
    case RequestInfo::State::enqueueable:
        rinfo->enqueue(connection);
        break;

    case RequestInfo::State::fresh:
        rinfo->state = RequestInfo::State::handling;
        return static_cast<Http::Detail*>(cls)->request(connection, rinfo);

    default:
        break;
    }

    return MHD_YES;
}

void mapproxy_http_callback_error(void*, const char *fmt, va_list ap)
{
    char buf[1024];
    ::vsnprintf(buf, sizeof(buf) - 1, fmt, ap);
    buf[sizeof(buf) - 1] = '\0';
    setLastError(buf);
}

} // extern "C"

Http::Detail::Detail(const utility::TcpEndpoint &listen
                     , unsigned int threadCount
                     , ContentGenerator &contentGenerator)
    : contentGenerator(contentGenerator)
    , daemon(::MHD_start_daemon
             ((MHD_USE_POLL_INTERNALLY | MHD_USE_SUSPEND_RESUME
               | MHD_USE_DEBUG)
              , listen.value.port()
              , nullptr, nullptr
              , &mapproxy_http_callback_request, this
              // TODO: get sock addr from endpoint
              // , MHD_OPTION_SOCK_ADDR, ???
              , MHD_OPTION_THREAD_POOL_SIZE
              , (unsigned int)(threadCount)

              , MHD_OPTION_NOTIFY_COMPLETED
              , &mapproxy_http_callback_completed, this

              , MHD_OPTION_EXTERNAL_LOGGER
              , &mapproxy_http_callback_error, this

              , MHD_OPTION_END))
{
    CHECK_MHD_ERROR(daemon, "Cannot start HTTP daemon");
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
    HttpSink(::MHD_Connection *connection, RequestInfo *ri)
        : conn_(connection), riRaw_(RequestInfo::addRef(ri))
        , ri_(*ri)
    {}

    ~HttpSink() { RequestInfo::decRef(riRaw_); }

private:
    virtual void content_impl(const void *data, std::size_t size
                              , const FileInfo &stat, bool needCopy)
    {
        if (!valid()) { return; }

        ri_.response.buffer(data, size, needCopy);
        CHECK_MHD_ERROR(::MHD_add_response_header
                        (ri_.response.get(), MHD_HTTP_HEADER_CONTENT_TYPE
                         , stat.contentType.c_str())
                        , "Unable to set response header");
        CHECK_MHD_ERROR(::MHD_add_response_header
                        (ri_.response.get(), MHD_HTTP_HEADER_LAST_MODIFIED
                         , formatHttpDate(stat.lastModified).c_str())
                         , "Unable to set response header");

        enqueue();
    }

    virtual void seeOther_impl(const std::string &url)
    {
        if (!valid()) { return; }

        // TODO: compose body
        ri_.response.buffer("", false);

        ri_.responseCode = MHD_HTTP_SEE_OTHER;
        ri_.location = url;

        CHECK_MHD_ERROR(::MHD_add_response_header
                        (ri_.response.get(), MHD_HTTP_HEADER_LOCATION
                         , url.c_str())
                        , "Unable to set response header");
        enqueue();
    }

    virtual void listing_impl(const Listing &list)
    {
        if (!valid()) { return; }

        std::ostringstream os;
        os << R"RAW(<html>
<head><title>Index of )RAW" << ri_.url
           << R"RAW(</title></head>
<body bgcolor="white">
<h1>Index of )RAW"
           << ri_.url
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

        const std::string *body;
        try {
            std::rethrow_exception(exc);
        } catch (const NotFound &e) {
            ri_.responseCode = MHD_HTTP_NOT_FOUND;
            ri_.errorReason = e.what();
            body = &error404;
        } catch (const NotAllowed &e) {
            ri_.responseCode = MHD_HTTP_METHOD_NOT_ALLOWED;
            ri_.errorReason = e.what();
            body = &error405;
        } catch (const Unavailable &e) {
            ri_.responseCode = MHD_HTTP_SERVICE_UNAVAILABLE;
            ri_.errorReason = e.what();
            body = &error503;
        } catch (const std::exception &e) {
            ri_.responseCode = MHD_HTTP_INTERNAL_SERVER_ERROR;
            ri_.errorReason = e.what();
            body = &error500;
        } catch (...) {
            ri_.responseCode = MHD_HTTP_INTERNAL_SERVER_ERROR;
            ri_.errorReason = "Unknown exception caught.";
            body = &error500;
        }

        LOG(debug) << "About to send http error: <" << ri_.errorReason << ">.";

        // enqueue response with proper status and body; body is static string
        // and thus not copied
        ri_.response.buffer(*body, false);

        CHECK_MHD_ERROR(::MHD_add_response_header
                        (ri_.response.get(), MHD_HTTP_HEADER_CONTENT_TYPE
                         , "text/html")
                        , "Unable to set response header");
        enqueue();
    }

    bool finished() const {
        return (ri_.state == RequestInfo::State::finished);
    }

    bool checkAborted_impl() const {
        return finished();
    }

    bool valid() const {
        if (finished()) { return false; }

        if (ri_.state != RequestInfo::State::handling) {
            LOG(warn3)
                << "Logic error in your code: attempt to send "
                "another response while it was already sent.";
            return false;
        }
        return true;
    }

    void enqueue() {
        ri_.enqueue(conn_);
    }

    ::MHD_Connection *conn_;
    RequestInfo *riRaw_;
    RequestInfo &ri_;
};

} // namespace

int Http::Detail::request(::MHD_Connection *connection
                          , RequestInfo *ri)
{
    auto sink(std::make_shared<HttpSink>(connection, ri));
    try {
        if ((ri->method == "HEAD") || (ri->method == "GET")) {
            contentGenerator.generate(ri->url, sink);
        } else {
            sink->error(utility::makeError<NotAllowed>
                        ("Method %s is not supported.", ri->method));
        }
    } catch (...) {
        sink->error();
    }

    return MHD_YES;
}

Http::Http(const utility::TcpEndpoint &listen, unsigned int threadCount
           , ContentGenerator &contentGenerator)
    : detail_(std::make_shared<Detail>(listen, threadCount, contentGenerator))
{
}
