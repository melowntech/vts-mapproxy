#include <ctime>

#include <boost/noncopyable.hpp>

#include <arpa/inet.h>

#include <microhttpd.h>

#include "dbglog/dbglog.hpp"

#include "utility/raise.hpp"

#include "./error.hpp"
#include "./http.hpp"

namespace {

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

struct RequestInfo {
    std::string url;
    std::string method;
    std::string version;
    int responseCode;
    std::string errorReason;

    RequestInfo(const std::string &url, const std::string &method
                , const std::string &version)
        : url(url), method(method), version(version)
        , responseCode(MHD_HTTP_OK)
    {}
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

    int request(::MHD_Connection *connection, RequestInfo &requestInfo);

    ContentGenerator &contentGenerator;
    ::MHD_Daemon *daemon;
};

extern "C" {

void mapproxy_http_callback_completed(void *cls, ::MHD_Connection *connection
                                      , void **info
                                      , enum MHD_RequestTerminationCode toe)
{
    (void) cls;

    // put into unique ptr to ensude deletion
    std::unique_ptr<RequestInfo> pinfo(static_cast<RequestInfo*>(*info));
    auto &rinfo(*pinfo);

    switch (toe) {
    case MHD_REQUEST_TERMINATED_COMPLETED_OK:
        if (rinfo.responseCode == MHD_HTTP_OK) {
            LOG(info3) << "HTTP " << ClientInfo(connection) << ' ' << rinfo
                       << ' ' << rinfo.responseCode << ".";
        } else {
            LOG(err2) << "HTTP " << ClientInfo(connection) << ' ' << rinfo
                      << ' ' << rinfo.responseCode << "; reason: <"
                      << rinfo.errorReason << ">.";
        }
        return;

    case MHD_REQUEST_TERMINATED_WITH_ERROR:
        LOG(err2) << "HTTP " << ClientInfo(connection) << ' ' << rinfo
                  << " [internal error].";
        return;

    case MHD_REQUEST_TERMINATED_TIMEOUT_REACHED:
        LOG(err2) << "HTTP " << ClientInfo(connection) << ' ' << rinfo
                  << " [timed-out].";
        return;

    case MHD_REQUEST_TERMINATED_DAEMON_SHUTDOWN:
        LOG(err2) << "HTTP " << ClientInfo(connection) << ' ' << rinfo
                  << " [shutdown].";
        return;

    case MHD_REQUEST_TERMINATED_READ_ERROR:
        LOG(err2)<< "HTTP " << ClientInfo(connection) << ' ' << rinfo
                  << " [read error].";
        return;

    case MHD_REQUEST_TERMINATED_CLIENT_ABORT:
        LOG(err2) << "HTTP " << ClientInfo(connection) << ' ' << rinfo
                  << " [aborted].";
        return;
    }
}

int mapproxy_http_callback_request(void *cls, ::MHD_Connection *connection
                                   , const char *url, const char *method
                                   , const char *version
                                   , const char *, size_t *, void **info)
{
    dbglog::thread_id("http");
    if (!*info) {
        // setup, log and done
        auto *rinfo(new RequestInfo(url, method, version));
        *info = rinfo;

        // received request logging
        LOG(info2) << "HTTP " << ClientInfo(connection)  << ' ' << *rinfo
                   << ".";

        return MHD_YES;
    }

    auto *rinfo(static_cast<RequestInfo*>(*info));

    // second call -> process
    return static_cast<Http::Detail*>(cls)->request
        (connection, *rinfo);
}

} // extern "C"

Http::Detail::Detail(const utility::TcpEndpoint &listen
                     , unsigned int threadCount
                     , ContentGenerator &contentGenerator)
    : contentGenerator(contentGenerator)
    , daemon(::MHD_start_daemon
             ((MHD_USE_POLL_INTERNALLY | MHD_USE_SUSPEND_RESUME)
              , listen.value.port()
              , nullptr, nullptr
              , &mapproxy_http_callback_request, this
              // TODO: get sock addr from endpoint
              // , MHD_OPTION_SOCK_ADDR, ???
              , MHD_OPTION_THREAD_POOL_SIZE
              , (unsigned int)(threadCount)

              , MHD_OPTION_NOTIFY_COMPLETED
              , &mapproxy_http_callback_completed, this

              , MHD_OPTION_END))
{
    if (!daemon) {
        LOGTHROW(err1, Error) << "Cannot start HTTP daemon.";
    }
}

namespace {

class ResponseHandle {
public:
    ResponseHandle() : response_() {}
    ResponseHandle(const std::string &data, bool copy = true) {
        buffer(data, copy);
    }

    ~ResponseHandle() {
        if (response_) { ::MHD_destroy_response(response_); }
    }

    void buffer(const std::string &data, bool copy = true) {
        response_ = ::MHD_create_response_from_buffer
            (data.size(), const_cast<char*>(data.data())
             ,  copy ? MHD_RESPMEM_MUST_COPY : MHD_RESPMEM_PERSISTENT);
    }

    MHD_Response* get() { return response_; }
    operator MHD_Response*() { return get(); }

private:
    MHD_Response *response_;
};

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
    HttpSink(::MHD_Connection *connection, RequestInfo &ri)
        : conn_(connection), ri_(ri)
    {}

private:
    virtual void content_impl(const std::string &data
                              , const FileInfo &stat)
    {
        ResponseHandle response(data);
        ::MHD_add_response_header
              (response.get(), MHD_HTTP_HEADER_CONTENT_TYPE
               , stat.contentType.c_str());
        ::MHD_add_response_header
              (response.get(), MHD_HTTP_HEADER_LAST_MODIFIED
               , formatHttpDate(stat.lastModified).c_str());

        ::MHD_queue_response(conn_, (ri_.responseCode = MHD_HTTP_OK)
                             , response);
    }

    virtual void error_impl(const std::exception_ptr &exc)
    {
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

        // enqueue response with proper status and body; body is static string
        // and this not copied
        ::MHD_queue_response(conn_, ri_.responseCode
                             , ResponseHandle(*body, false));
    }

    ::MHD_Connection *conn_;
    RequestInfo &ri_;
};

} // namespace

int Http::Detail::request(::MHD_Connection *connection
                          , RequestInfo &ri)
{
    auto sink(std::make_shared<HttpSink>(connection, ri));
    try {
        if ((ri.method == "HEAD") || (ri.method == "GET")) {
            contentGenerator.generate(ri.url, sink);
        } else {
            sink->error(utility::makeError<NotAllowed>
                        ("Method %s is not supported.", ri.method));
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
