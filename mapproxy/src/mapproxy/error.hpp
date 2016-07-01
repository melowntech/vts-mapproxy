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

/** Abandon all operations.
 */
struct AbandonAll : Error {
    AbandonAll(const std::string &message) : Error(message) {}
};

class GenerateError : public Error {
public:
    typedef void(*RaiseError)(const std::string &message);

    GenerateError(const std::string &message, RaiseError raiseError)
        : Error(message), raiseError_(raiseError)
    {}

    RaiseError getRaise() const { return raiseError_; }

    static void runtimeError(const std::string &message) {
        throw std::runtime_error(message);
    }

private:
    RaiseError raiseError_;
};

#define ERROR_GENERATE_ERROR(Type)                          \
    struct Type : GenerateError {                           \
        Type(const std::string &message)                    \
            : GenerateError(message, &Type::raiseImpl) {}   \
        static void raiseImpl(const std::string &message) { \
            throw Type(message);                            \
        }                                                   \
    }

/** Given URL does not exist.
 */
ERROR_GENERATE_ERROR(NotFound);

/** Given URL is not available now.
 */
ERROR_GENERATE_ERROR(Unavailable);

/** Internal Error.
 */
ERROR_GENERATE_ERROR(InternalError);

/** Request aborted
 */
ERROR_GENERATE_ERROR(RequestAborted);

/** Image to be sent is empty. Translated to some defined content.
 */
ERROR_GENERATE_ERROR(EmptyImage);

#endif // mapproxy_error_hpp_included_
