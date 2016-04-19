#ifndef mapproxy_contentgenerator_hpp_included_
#define mapproxy_contentgenerator_hpp_included_

#include <ctime>
#include <string>
#include <memory>
#include <exception>

#include "vts-libs/storage/streams.hpp"

namespace vs = vadstena::storage;

/** Sink for sending data to the client.
 */
class Sink {
public:
    typedef std::shared_ptr<Sink> pointer;

    typedef std::function<void()> AbortedCallback;

    Sink() {}
    virtual ~Sink() {}

    struct FileInfo {
        /** File content type.
         */
        std::string contentType;

        /** Timestamp of last modification. -1 means now.
         */
        std::time_t lastModified;

        FileInfo(const std::string &contentType = "application/octet-stream"
                 , std::time_t lastModified = -1)
            : contentType(contentType), lastModified(lastModified)
        {}
    };

    struct ListingItem {
        enum class Type { dir, file };

        std::string name;
        Type type;

        ListingItem(const std::string &name = "", Type type = Type::file)
            : name(name), type(type)
        {}

        bool operator<(const ListingItem &o) const;
    };

    typedef std::vector<ListingItem> Listing;

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
     */
    void content(const vs::IStream::pointer &stream);

    /** Tell client to look somewhere else.
     */
    void seeOther(const std::string &url);

    /** Generates listing.
     */
    void listing(const Listing &list);

    /** Sends current exception to the client.
     */
    void error();

    /** Sends given error to the client.
     */
    void error(const std::exception_ptr &exc);

    /** Sends given error to the client.
     */
    template <typename T> void error(const T &exc);

    /** Checks wheter client aborted request.
     *  Throws RequestAborted exception when true.
     */
    void checkAborted() const;

    /** Sets aborted callback.
     */
    void setAborter(const AbortedCallback &ac);

private:
    virtual void content_impl(const void *data, std::size_t size
                              , const FileInfo &stat, bool needCopy) = 0;
    virtual void content_impl(const vs::IStream::pointer &stream) = 0;
    virtual void seeOther_impl(const std::string &url) = 0;
    virtual void listing_impl(const Listing &list) = 0;
    virtual void error_impl(const std::exception_ptr &exc) = 0;
    virtual bool checkAborted_impl() const = 0;
    virtual void setAborter_impl(const AbortedCallback &ac) = 0;
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
    content_impl(data.data(), data.size(), stat, true);
}

inline void Sink::content(const void *data, std::size_t size
                          , const FileInfo &stat, bool needCopy)
{
    content_impl(data, size, stat, needCopy);
}

template <typename T>
inline void Sink::content(const std::vector<T> &data, const FileInfo &stat)
{
    content_impl(data.data(), data.size() * sizeof(T), stat, true);
}

inline void Sink::content(const vs::IStream::pointer &stream)
{
    content_impl(stream);
}

inline void Sink::seeOther(const std::string &url) { seeOther_impl(url); }

inline void Sink::error(const std::exception_ptr &exc) { error_impl(exc); }

template <typename T>
inline void Sink::error(const T &exc)
{
    try {
        throw exc;
    } catch (...) {
        error();
    }
}

inline void Sink::listing(const Listing &list) { listing_impl(list); }

inline void Sink::setAborter(const AbortedCallback &ac) {
    setAborter_impl(ac);
}

inline void ContentGenerator::generate(const std::string &location
                                       , const Sink::pointer &sink)
{
    return generate_impl(location, sink);
}

inline bool Sink::ListingItem::operator<(const ListingItem &o) const
{
    if (type < o.type) { return true; }
    if (o.type < type) { return false; }
    return name < o.name;
}

#endif // mapproxy_contentgenerator_hpp_included_
