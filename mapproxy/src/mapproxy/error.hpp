#ifndef mapproxy_error_hpp_included_
#define mapproxy_error_hpp_included_

#include <stdexcept>
#include <string>

#include "http/error.hpp"

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

/** Abandon all operations.
 */
struct AbandonAll : Error {
    AbandonAll(const std::string &message) : Error(message) {}
};

struct EmptyImage : Error {
    EmptyImage(const std::string &message) : Error(message) {}
};

typedef http::NotFound NotFound;
typedef http::ServiceUnavailable Unavailable;
typedef http::InternalServerError InternalError;
typedef http::RequestAborted RequestAborted;
typedef http::BadRequest BadRequest;

#endif // mapproxy_error_hpp_included_
