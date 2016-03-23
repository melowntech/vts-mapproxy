#ifndef mapproxy_error_hpp_included_
#define mapproxy_error_hpp_included_

#include <stdexcept>
#include <string>

struct Error : std::runtime_error {
    Error(const std::string &message) : std::runtime_error(message) {}
};

struct UnknownResourceBackend : Error {
    UnknownResourceBackend(const std::string &message) : Error(message) {}
};

struct UnknownGenerator : Error {
    UnknownGenerator(const std::string &message) : Error(message) {}
};

struct InvalidConfiguration : Error {
    InvalidConfiguration(const std::string &message) : Error(message) {}
};

struct IOError : Error {
    IOError(const std::string &message) : Error(message) {}
};

struct FormatError : Error {
    FormatError(const std::string &message) : Error(message) {}
};

struct NotFound : Error {
    NotFound(const std::string &message) : Error(message) {}
};

#endif // mapproxy_error_hpp_included_
