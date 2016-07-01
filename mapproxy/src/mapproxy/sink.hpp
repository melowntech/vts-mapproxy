#ifndef mapproxy_sink_hpp_included_
#define mapproxy_sink_hpp_included_

#include <ctime>
#include <string>
#include <memory>
#include <exception>

#include "http/contentgenerator.hpp"

#include "vts-libs/storage/streams.hpp"

namespace vs = vadstena::storage;

/** Wraps libhttp's sink.
 */
class Sink {
public:
    typedef http::Sink::AbortedCallback AbortedCallback;
    typedef http::Sink::FileInfo FileInfo;
    typedef http::Sink::ListingItem ListingItem;
    typedef std::vector<ListingItem> Listing;

    Sink(const http::Sink::pointer &sink) : sink_(sink) {}

    /** Sends content to client.
     * \param data data top send
     * \param stat file info (size is ignored)
     */
    void content(const std::string &data, const FileInfo &stat) {
        sink_->content(data, stat);
    }

    /** Sends content to client.
     * \param data data top send
     * \param stat file info (size is ignored)
     */
    template <typename T>
    void content(const std::vector<T> &data, const FileInfo &stat) {
        sink_->content(data, stat);
    }

    /** Sends content to client.
     * \param data data top send
     * \param size size of data
     * \param stat file info (size is ignored)
     * \param needCopy data are copied if set to true
     */
    void content(const void *data, std::size_t size
                 , const FileInfo &stat, bool needCopy)
    {
        sink_->content(data, size, stat, needCopy);
    }

    /** Sends content to client.
     * \param stream stream to send
     */
    void content(const vs::IStream::pointer &stream);

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
    void setAborter(const AbortedCallback &ac) {
        sink_->setAborter(ac);
    }

private:
    /** Sends given error to the client.
     */
    void error(const std::exception_ptr &exc);

    http::Sink::pointer sink_;
};

template <typename T>
inline void Sink::error(const T &exc)
{
    error(std::make_exception_ptr(exc));
}

inline void Sink::error() {error(std::current_exception()); }

#endif // mapproxy_sink_hpp_included_

