#ifndef mapproxy_http_hpp_included_
#define mapproxy_http_hpp_included_

#include <memory>
#include <string>

#include "utility/tcpendpoint.hpp"

#include "contentgenerator.hpp"

class Http {
public:
    Http(const utility::TcpEndpoint &listen
         , unsigned int threadCount
         , ContentGenerator &contentGenerator);

    struct Detail;

private:
    std::shared_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};

#endif // mapproxy_http_hpp_included_
