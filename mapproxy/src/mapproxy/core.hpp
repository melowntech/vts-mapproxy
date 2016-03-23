#ifndef mapproxy_core_hpp_included_
#define mapproxy_core_hpp_included_

#include "./contentgenerator.hpp"
#include "./generator.hpp"

class Core : boost::noncopyable
           , public ContentGenerator
{
public:
    Core(Generators &generators);

    struct Detail;

private:
    virtual void generate_impl(const std::string &location
                               , const Sink::pointer &sink);

    std::shared_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};

#endif // mapproxy_core_hpp_included_
