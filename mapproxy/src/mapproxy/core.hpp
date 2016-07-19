#ifndef mapproxy_core_hpp_included_
#define mapproxy_core_hpp_included_

#include "http/contentgenerator.hpp"

#include "./generator.hpp"

class Core : boost::noncopyable
           , public http::ContentGenerator
{
public:
    Core(Generators &generators, GdalWarper &warper, unsigned int threadCount
         , http::ContentFetcher &contentFetcher);

    struct Detail;

private:
    virtual void generate_impl(const std::string &location
                               , const http::ServerSink::pointer &sink);

    std::shared_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};

#endif // mapproxy_core_hpp_included_
