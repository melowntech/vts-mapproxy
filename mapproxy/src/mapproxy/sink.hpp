#ifndef mapproxy_sink_hpp_included_
#define mapproxy_sink_hpp_included_

#include <ctime>
#include <string>
#include <memory>
#include <exception>

#include "dbglog/dbglog.hpp"

#include "utility/enum-io.hpp"

#include "http/contentgenerator.hpp"

#include "vts-libs/storage/streams.hpp"

namespace vs = vadstena::storage;

/** Aborter helper.
 */
struct Aborter {
    typedef http::ServerSink::AbortedCallback AbortedCallback;

    virtual ~Aborter() {}

    /** Defaults to dummy aborter
     */
    virtual void setAborter(const AbortedCallback&) {};
};

/** Wraps libhttp's sink.
 */
class Sink : public Aborter {
public:
    typedef http::ServerSink::AbortedCallback AbortedCallback;
    typedef http::ServerSink::ListingItem ListingItem;
    typedef std::vector<ListingItem> Listing;

    struct FileInfo : http::SinkBase::FileInfo {
        enum class FileClass {
            unknown, config, browser, support, registry, data
        };

        FileInfo(const std::string &contentType = "application/octet-stream"
                 , std::time_t lastModified = -1
                 , std::time_t expires = -1)
            : http::SinkBase::FileInfo(contentType, lastModified, expires)
        {}

        FileInfo& setFileClass(FileClass fc);

        http::Header::list headers;
    };

    Sink(const http::ServerSink::pointer &sink) : sink_(sink) {}

    /** Sends content to client.
     * \param data data top send
     * \param stat file info (size is ignored)
     */
    void content(const std::string &data, const FileInfo &stat);

    /** Sends content to client.
     * \param data data top send
     * \param stat file info (size is ignored)
     */
    template <typename T>
    void content(const std::vector<T> &data, const FileInfo &stat);

    /** Sends content to client.
     * \param data data top send
     * \param size size of data
     * \param stat file info (size is ignored)
     * \param needCopy data are copied if set to true
     */
    void content(const void *data, std::size_t size
                 , const FileInfo &stat, bool needCopy);

    /** Sends content to client.
     * \param stream stream to send
     * \param file class
     */
    void content(const vs::IStream::pointer &stream
                 , FileInfo::FileClass fileClass);

    /** Tell client to look somewhere else.
     */
    void seeOther(const std::string &url) {
        sink_->seeOther(url);
    }

    /** Generates listing.
     */
    void listing(const Listing &list) {
        sink_->listing(list);
    }

    /** Sends current exception to the client.
     */
    void error();

    /** Sends given error to the client.
     */
    template <typename T> void error(const T &exc);

    /** Checks wheter client aborted request.
     *  Throws RequestAborted exception when true.
     */
    void checkAborted() const {
        sink_->checkAborted();
    }

    /** Sets aborted callback.
     */
    virtual void setAborter(const AbortedCallback &ac) {
        sink_->setAborter(ac);
    }

private:
    /** Sends given error to the client.
     */
    void error(const std::exception_ptr &exc);

    http::ServerSink::pointer sink_;
};

// inlines

template <typename T>
inline void Sink::error(const T &exc)
{
    error(std::make_exception_ptr(exc));
}

inline void Sink::error() { error(std::current_exception()); }

inline void Sink::content(const std::string &data, const FileInfo &stat) {
    sink_->content(data, stat, &stat.headers);
}

template <typename T>
inline void Sink::content(const std::vector<T> &data, const FileInfo &stat) {
    sink_->content(data, stat, &stat.headers);
}

inline void Sink::content(const void *data, std::size_t size
                          , const FileInfo &stat, bool needCopy)
{
    sink_->content(data, size, stat, needCopy, &stat.headers);
}

UTILITY_GENERATE_ENUM_IO(Sink::FileInfo::FileClass,
    ((unknown))
    ((config))
    ((browser))
    ((support))
    ((registry))
    ((data))
)

#endif // mapproxy_sink_hpp_included_

