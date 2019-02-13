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

#ifndef mapproxy_support_fileclass_hpp_included_
#define mapproxy_support_fileclass_hpp_included_

#include <array>

#include <boost/any.hpp>
#include <boost/program_options.hpp>

#include "utility/enum-io.hpp"

/** If adding into this enum leave unknown the last one!
 *  Make no holes, i.e. we can use values directly as indices to an array
 */
enum class FileClass { config, support, registry, data, unknown };

class FileClassSettings {
public:
    static constexpr std::size_t storageSize =
        static_cast<int>(FileClass::unknown) + 1;
    typedef std::array<std::time_t, storageSize> Times;

    FileClassSettings() : maxAges_{{0}}, staleWhileRevalidate_{{0}} {
        // unknown files are never cached -- for example directory listings
        setMaxAge(FileClass::unknown, -1);
    }

    void from(const boost::any &value);
    void to(boost::any &value) const;

    void configuration(boost::program_options::options_description &od
                              , const std::string &prefix = "");

    void setMaxAge(FileClass fc, long value);
    long getMaxAge(FileClass fc) const;

    void setStaleWhileRevalidate(FileClass fc, std::time_t value);
    std::time_t getStaleWhileRevalidate(FileClass fc) const;

private:
    Times maxAges_;
    Times staleWhileRevalidate_;
};

UTILITY_GENERATE_ENUM_IO(FileClass,
                         ((config))
                         ((support))
                         ((registry))
                         ((data))
                         ((unknown))
                         )

// inlines

inline void FileClassSettings::setMaxAge(FileClass fc, long value)
{
    maxAges_[static_cast<int>(fc)] = value;
}

inline long FileClassSettings::getMaxAge(FileClass fc) const
{
    return maxAges_[static_cast<int>(fc)];
}

inline void
FileClassSettings::setStaleWhileRevalidate(FileClass fc, std::time_t value)
{
    staleWhileRevalidate_[static_cast<int>(fc)] = value;
}

inline std::time_t FileClassSettings::getStaleWhileRevalidate(FileClass fc)
    const
{
    return staleWhileRevalidate_[static_cast<int>(fc)];
}

#endif // mapproxy_support_fileclass_hpp_included_
