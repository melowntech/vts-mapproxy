#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "http/error.hpp"

#include "./sink.hpp"
#include "./error.hpp"

namespace {

http::Header::list buildHeaders(const Sink::FileInfo &stat
                                , const FileClassSettings *fileClassSettings)
{
    auto headers(stat.headers);

    // TODO: obsolete, remove
    headers.emplace_back("X-MapProxy-File-Class"
                         , boost::lexical_cast<std::string>(stat.fileClass));

    if (!fileClassSettings) {
        // no file class attached, no caching
        headers.emplace_back("Cache-Control", "no-cache");
        return headers;
    }

    // set max age based on fileClass settings
    auto maxAge(fileClassSettings->getMaxAge(stat.fileClass));
    if (maxAge < 0) {
        headers.emplace_back("Cache-Control", "no-cache");
    } else {
        headers.emplace_back("Cache-Control"
                             , str(boost::format("max-age=%s")
                                   % maxAge));
    }

    return headers;
}

const std::vector<unsigned char> emptyImage([]() -> std::vector<unsigned char>
{
    cv::Mat dot(4, 4, CV_8U, cv::Scalar(0));
    std::vector<unsigned char> buf;
    cv::imencode(".png", dot, buf
                 , { cv::IMWRITE_PNG_COMPRESSION, 9 });
    return buf;
}());

class IStreamDataSource : public http::ServerSink::DataSource {
public:
    IStreamDataSource(const vs::IStream::pointer &stream
                      , FileClass fileClass
                      , const FileClassSettings *fileClassSettings)
        : stream_(stream), stat_(stream->stat())
        , fs_(Sink::FileInfo(stat_.contentType, stat_.lastModified)
              .setFileClass(fileClass))
        , headers_(buildHeaders(fs_, fileClassSettings))
    {
        // do not fail on eof
        stream->get().exceptions(std::ios::badbit);
    }

    virtual http::SinkBase::FileInfo stat() const {
        return {  };
    }

    virtual std::size_t read(char *buf, std::size_t size
                                 , std::size_t off)
    {
        return stream_->read(buf, size, off);
    }

    virtual std::string name() const { return stream_->name(); }

    virtual void close() const { stream_->close(); }

    virtual long size() const { return stat_.size; }

    virtual const http::Header::list *headers() const { return &headers_; }

private:
    vs::IStream::pointer stream_;
    vs::FileStat stat_;
    Sink::FileInfo fs_;
    http::Header::list headers_;
};

} //namesapce

void Sink::content(const vs::IStream::pointer &stream
                   , FileClass fileClass)
{
    sink_->content(std::make_shared<IStreamDataSource>
                   (stream, fileClass, fileClassSettings_));
}

void Sink::error(const std::exception_ptr &exc)
{
    try {
        std::rethrow_exception(exc);
    } catch (const EmptyImage &e) {
        // special "error" -> send "empty" image
        content(emptyImage.data(), emptyImage.size()
                , Sink::FileInfo("image/png")
                .setFileClass(FileClass::data)
                , false);
    } catch (...) {
        sink_->error(std::current_exception());
    }
}

Sink::FileInfo& Sink::FileInfo::setFileClass(FileClass fc) {
    fileClass = fc;
    return *this;
}

http::Header::list Sink::buildHeaders(const FileInfo &stat) const
{
    return ::buildHeaders(stat, fileClassSettings_);
}
