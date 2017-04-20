/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"

#include "imgproc/png.hpp"

#include "http/error.hpp"

#include "vts-libs/vts/2d.hpp"

#include "./sink.hpp"
#include "./error.hpp"

namespace vts = vtslibs::vts;

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

const auto fullImage([]() -> std::vector<unsigned char>
{
    cv::Mat dot(8, 8, CV_8U, cv::Scalar(255));
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
    } catch (const FullImage &e) {
        // special "error" -> send "full" image
        content(fullImage.data(), fullImage.size()
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
        // special "error" -> send "empty" geodata
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
