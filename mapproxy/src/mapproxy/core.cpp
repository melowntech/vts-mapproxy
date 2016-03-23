#include "./fileinfo.hpp"

#include "./core.hpp"

struct Core::Detail : boost::noncopyable {
    Detail(Generators &generators)
        : generators(generators)
    {}

    void generate(const std::string &location
                  , const Sink::pointer &sink);

    Generators &generators;
};

Core::Core(Generators &generators)
    : detail_(std::make_shared<Detail>(generators))
{}

void Core::generate_impl(const std::string &location
                         , const Sink::pointer &sink)
{
    detail().generate(location, sink);
}

void Core::Detail::generate(const std::string &location
                            , const Sink::pointer &sink)
{
    FileInfo fi(location);

    (void) fi;
    (void) sink;
}
