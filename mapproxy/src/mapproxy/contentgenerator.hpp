#ifndef mapproxy_contentgenerator_hpp_included_
#define mapproxy_contentgenerator_hpp_included_

#include <string>
#include <memory>
#include <exception>

#include "./fileinfo.hpp"

class Sink {
public:
    typedef std::shared_ptr<Sink> pointer;

    Sink() {}
    virtual ~Sink() {}

    void content(const std::string &data);
    void error(const std::exception_ptr &exc);

private:
    virtual void content_impl(const std::string &data) = 0;
    virtual void error_impl(const std::exception_ptr &exc) = 0;
};

class ContentGenerator {
public:
    virtual ~ContentGenerator() {}
    void generate(const std::string &location
                  , const Sink::pointer &sink);

private:
    virtual void generate_impl(const std::string &location
                               , const Sink::pointer &sink) = 0;
};

// inlines

inline void Sink::content(const std::string &data)
{
    content_impl(data);
}

inline void Sink::error(const std::exception_ptr &exc)
{
    error_impl(exc);
}

inline void ContentGenerator::generate(const std::string &location
                                       , const Sink::pointer &sink)
{
    return generate_impl(location, sink);
}

#endif // mapproxy_contentgenerator_hpp_included_
