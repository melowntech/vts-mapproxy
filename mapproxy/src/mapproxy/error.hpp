#ifndef mapproxy_error_hpp_included_
#define mapproxy_error_hpp_included_

#include <stdexcept>
#include <string>

struct Error : std::runtime_error {
    Error(const std::string &message) : std::runtime_error(message) {}
};

struct UnknownResourceBackend : std::runtime_error {
    UnknownResourceBackend(const std::string &message)
        : std::runtime_error(message) {}
};

struct InvalidConfiguration : std::runtime_error {
    InvalidConfiguration(const std::string &message)
        : std::runtime_error(message) {}
};

#endif // mapproxy_error_hpp_included_
