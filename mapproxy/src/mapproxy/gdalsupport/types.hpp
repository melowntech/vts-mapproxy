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

#ifndef mapproxy_gdalsupport_types_hpp_included_
#define mapproxy_gdalsupport_types_hpp_included_

#include <boost/interprocess/smart_ptr/shared_ptr.hpp>
#include <boost/interprocess/smart_ptr/weak_ptr.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/smart_ptr/deleter.hpp>
#include <boost/interprocess/managed_external_buffer.hpp>
#include <boost/interprocess/indexes/flat_map_index.hpp>
#include <boost/interprocess/anonymous_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/containers/deque.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <boost/date_time/microsec_time_clock.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

namespace bi = boost::interprocess;

typedef bi::basic_managed_external_buffer<
    char
    , bi::rbtree_best_fit<bi::mutex_family, void*>
    , bi::flat_map_index> ManagedBuffer;

typedef ManagedBuffer::segment_manager SegmentManager;

typedef bi::allocator<void, SegmentManager> Allocator;

typedef bi::basic_string<
    char, std::char_traits<char>
    , bi::allocator<char, SegmentManager>
    > String;


typedef bi::vector<String, bi::allocator<String, SegmentManager>> StringVector;

typedef bi::scoped_lock<bi::interprocess_mutex> Lock;

inline std::string asString(const String &str) {
    return { str.data(), str.size() };
}

inline boost::optional<std::string> asOptional(const String &str) {
    if (str.empty()) { return {}; }
    return std::string(str.data(), str.size());
}

inline boost::optional<geo::SrsDefinition>
asOptional(const String &str, geo::SrsDefinition::Type type)
{
    if (str.empty()) { return {}; }
    return geo::SrsDefinition(std::string(str.data(), str.size()), type);
}

struct ConstBlock {
    const char *data;
    std::size_t size;

    ConstBlock(const char *data = nullptr, std::size_t size = 0)
        : data(data), size(size)
    {}
};

#endif // mapproxy_gdalsupport_types_hpp_included_
