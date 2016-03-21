#ifndef mapproxy_generator_hpp_included_
#define mapproxy_generator_hpp_included_

#include <memory>
#include <string>
#include <iostream>

#include <boost/noncopyable.hpp>
#include <boost/any.hpp>

#include "./resources.hpp"

class Generator : boost::noncopyable {
public:
    typedef std::shared_ptr<Generator> pointer;

    virtual ~Generator() {}

    static Generator::pointer create(const std::string &type
                                     , const Resource &resource);

    struct Factory;
    static void registerType(const std::string &type
                             , const std::shared_ptr<Factory> &factory);

protected:
    Generator(const Resource &resource) : resource_(resource) {}
    const Resource resource_;
};

#endif // mapproxy_generator_hpp_included_
