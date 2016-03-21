#include <boost/noncopyable.hpp>

#include <arpa/inet.h>

#include <microhttpd.h>

#include "dbglog/dbglog.hpp"

#include "../http.hpp"

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
    Detail(const utility::TcpEndpoint &listen, unsigned int threadCount);
    ~Detail() {
        if (daemon) { ::MHD_stop_daemon(daemon); }
    }

    int request(::MHD_Connection *connection, RequestInfo &requestInfo);

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
    case MHD_REQUEST_TERMINATED_WITH_ERROR:
        LOG(err2) << "HTTP " << ClientInfo(connection) << ' ' << rinfo
                  << ' ' << rinfo.responseCode << ".";
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

} // extent "C"

Http::Detail::Detail(const utility::TcpEndpoint &listen
                     , unsigned int threadCount)
    : daemon(::MHD_start_daemon
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
}

int Http::Detail::request(::MHD_Connection *connection
                          , RequestInfo &requestInfo)
{
    (void) connection;
    (void) requestInfo;
    return MHD_YES;
}

Http::Http(const utility::TcpEndpoint &listen, unsigned int threadCount)
    : detail_(std::make_shared<Detail>(listen, threadCount))
{
}
