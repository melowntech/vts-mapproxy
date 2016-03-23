#ifndef mapproxy_core_hpp_included_
#define mapproxy_core_hpp_included_

#include "./contentgenerator.hpp"
#include "./generator.hpp"

class Core : boost::noncopyable
           , public ContentGenerator
{
public:
    Core(Generators &generators);

private:
    virtual void generate_impl(const std::string &location
                               , const Sink::pointer &sink);

    Generators &generators_;
};

#endif // mapproxy_core_hpp_included_
