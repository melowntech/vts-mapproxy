#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "imgproc/png.hpp"

#include "http/error.hpp"

#include "vts-libs/vts/2d.hpp"

#include "./sink.hpp"
#include "./error.hpp"

namespace vts = vadstena::vts;

namespace {

boost::optional<long> maxAge(FileClass fileClass
                             , const FileClassSettings *fileClassSettings
                             , const boost::optional<long> &forcedMaxAge)
{
    if (forcedMaxAge) { return forcedMaxAge; }
    if (!fileClassSettings) { return boost::none; }
    return fileClassSettings->getMaxAge(fileClass);
}

Sink::FileInfo update(const Sink::FileInfo &inStat
                      , const FileClassSettings *fileClassSettings)
{
    if (inStat.maxAge) { return inStat; }
    auto stat(inStat);

    if (!fileClassSettings) {
        // no file class attached, no caching
        stat.maxAge = -1;
        return stat;
    }

    // set max age based on fileClass settings
    stat.maxAge = fileClassSettings->getMaxAge(stat.fileClass);

    // done
    return stat;
}

const auto emptyImage([]() -> std::vector<unsigned char>
{
    cv::Mat dot(4, 4, CV_8U, cv::Scalar(0));
    std::vector<unsigned char> buf;
    cv::imencode(".png", dot, buf
                 , { cv::IMWRITE_PNG_COMPRESSION, 9 });
    return buf;
}());

const auto emptyDebugMask([]() -> std::vector<char>
{
    return imgproc::png::serialize(vts::emptyDebugMask(), 9);
}());

class IStreamDataSource : public http::ServerSink::DataSource {
public:
    IStreamDataSource(const vs::IStream::pointer &stream
                      , FileClass fileClass
                      , const FileClassSettings *fileClassSettings
                      , const boost::optional<long> &forcedMaxAge
                      , bool gzipped)
        : stream_(stream), stat_(stream->stat())
        , fs_(Sink::FileInfo(stat_.contentType, stat_.lastModified
                             , maxAge(fileClass, fileClassSettings
                                      , forcedMaxAge)))
    {
        // do not fail on eof
        stream->get().exceptions(std::ios::badbit);

        if (gzipped) {
            headers_.emplace_back("Content-Encoding", "gzip");
        }
    }

    virtual http::SinkBase::FileInfo stat() const {
        return fs_;
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

void Sink::content(const vs::IStream::pointer &stream, FileClass fileClass
                   , const boost::optional<long> &maxAge, bool gzipped)
{
    sink_->content(std::make_shared<IStreamDataSource>
                   (stream, fileClass, fileClassSettings_, maxAge, gzipped));
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
    } catch (const EmptyDebugMask &e) {
        // special "error" -> send "empty" image
        content(emptyDebugMask.data(), emptyDebugMask.size()
                , Sink::FileInfo("image/png")
                .setFileClass(FileClass::data)
                , false);
    } catch (const EmptyGeoData &e) {
        // special "error" -> send "empty" image
        content("{}", Sink::FileInfo("application/json; charset=utf-8")
                .setFileClass(FileClass::data));
    } catch (...) {
        sink_->error(std::current_exception());
    }
}

Sink::FileInfo& Sink::FileInfo::setFileClass(FileClass fc)
{
    fileClass = fc;
    return *this;
}

Sink::FileInfo& Sink::FileInfo::setMaxAge(const boost::optional<long> &ma)
{
    maxAge = ma;
    return *this;
}

Sink::FileInfo& Sink::FileInfo::addHeader(const std::string &name
                                          , const std::string &value)
{
    headers.emplace_back(name, value);
    return *this;
}

Sink::FileInfo Sink::update(const FileInfo &stat) const
{
    return ::update(stat, fileClassSettings_);
}
