#ifndef mapproxy_contentgenerator_hpp_included_
#define mapproxy_contentgenerator_hpp_included_

#include <ctime>
#include <string>
#include <memory>
#include <exception>

#include "vts-libs/storage/streams.hpp"

/** Sink for sending data to the client.
 */
class Sink {
public:
    typedef std::shared_ptr<Sink> pointer;

    Sink() {}
    virtual ~Sink() {}

    struct FileInfo {
        /** File content type.
         */
        std::string contentType;

        /** Timestamp of last modification. -1 means now.
         */
        std::time_t lastModified;

        FileInfo(const std::string &contentType, std::time_t lastModified = -1)
            : contentType(contentType), lastModified(lastModified)
        {}
    };

    /** Sends content to client.
     * \param data data top send
     * \param stat file info (size is ignored)
     */
    void content(const std::string &data, const FileInfo &stat);

    /** Sends current exception to the client.
     */
    void error();

    /** Sends given error to the client.
     */
    void error(const std::exception_ptr &exc);

    /** Sends given error to the client.
     */
    template <typename T> void error(const T &exc);

private:
    virtual void content_impl(const std::string &data
                              , const FileInfo &stat) = 0;
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

inline void Sink::content(const std::string &data, const FileInfo &stat)
{
    content_impl(data, stat);
}

inline void Sink::error(const std::exception_ptr &exc)
{
    error_impl(exc);
}

template <typename T>
inline void Sink::error(const T &exc)
{
    try {
        throw exc;
    } catch (...) {
        error_impl(std::current_exception());
    }
}

inline void Sink::error()
{
    error_impl(std::current_exception());
}

inline void ContentGenerator::generate(const std::string &location
                                       , const Sink::pointer &sink)
{
    return generate_impl(location, sink);
}

#endif // mapproxy_contentgenerator_hpp_included_
