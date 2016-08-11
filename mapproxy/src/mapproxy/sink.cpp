#include <opencv2/highgui/highgui.hpp>

#include "http/error.hpp"

#include "./sink.hpp"
#include "./error.hpp"

namespace {

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
    IStreamDataSource(const vs::IStream::pointer &stream)
        : stream_(stream), stat_(stream->stat())
    {
        // do not fail on eof
        stream->get().exceptions(std::ios::badbit);
    }

    virtual http::SinkBase::FileInfo stat() const {
        return { stat_.contentType, stat_.lastModified };
    }

    virtual std::size_t read(char *buf, std::size_t size
                                 , std::size_t off)
    {
        return stream_->read(buf, size, off);
    }

    virtual std::string name() const { return stream_->name(); }

    virtual void close() const { stream_->close(); }

    virtual long size() const { return stat_.size; }

private:
    vs::IStream::pointer stream_;
    vs::FileStat stat_;
};

} //namesapce

void Sink::content(const vs::IStream::pointer &stream)
{
    sink_->content(std::make_shared<IStreamDataSource>(stream));
}

const Sink::FileInfo emptyImageInfo("image/png");

void Sink::error(const std::exception_ptr &exc)
{
    try {
        std::rethrow_exception(exc);
    } catch (const EmptyImage &e) {
        // special "error" -> send "empty" image
        sink_->content(emptyImage.data(), emptyImage.size()
                       , emptyImageInfo, false);
    } catch (...) {
        sink_->error(std::current_exception());
    }
}
