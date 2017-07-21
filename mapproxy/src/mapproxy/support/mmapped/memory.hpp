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

#ifndef mapproxy_support_mmapped_memory_hpp_included_
#define mapproxy_support_mmapped_memory_hpp_included_

#include <array>
#include <iostream>

#include "vts-libs/vts/tileindex.hpp"
#include "vts-libs/vts/tileset/tilesetindex.hpp"

#include "./tileflags.hpp"

// #define MMAPTI_DEBUG

#ifdef MMAPTI_DEBUG
#include "dbglog/dbglog.hpp"
#endif

namespace vts = vtslibs::vts;

namespace mmapped {

/** Memory information.
 */
class Memory;

/** Memory reader
 */
class MemoryReader {
public:
    MemoryReader(const char *mem) : origin_(mem), mem_(mem) {}

    /** Read value of type T.
     */
    template <typename T> const T& read() {
        const T *ptr(reinterpret_cast<const T*>(mem_));
#ifdef MMAPTI_DEBUG
        LOG(info4) << "Reading from " << (void*)(ptr)
                   << " (offset: " << address(mem_) << ")";
#endif
        mem_ += sizeof(T);
        return *ptr;
    }

    /** Skip of count elements of size of T.
     */
    template <typename T> void skip(int count = 1) {
#ifdef MMAPTI_DEBUG
        LOG(info4)
            << "Skipping " << count << " * " << sizeof(T) << " bytes from "
            << (void*)(mem_) << " (offset: " << address(mem_) << ")";
#endif
        mem_ += (count * sizeof(T));
    }

    // read and apply jump value
    template <typename T> void jump() {
        const auto value(read<T>());
#ifdef MMAPTI_DEBUG
        LOG(info4) << "Jumped from " << (void*)(mem_)
                   << " (offset: " << address(mem_) << ")"
                   << " by " << value
                   << " to " << (void*)(mem_ + value)
                   << " (offset: " << address(mem_ + value) << ")";
#endif
        mem_ += value;
    }

    // read jump value and compute new address
    template <typename T> std::size_t jumpAddress() {
        const auto value(read<T>());
        return address() + value;
    }

    // rewind to given address
    void seek(std::size_t address) {
#ifdef MMAPTI_DEBUG
        LOG(info4)
            << "Seeking to " << address << ".";
#endif
        mem_ = origin_ + address;
    }

    std::size_t address() const { return mem_ - origin_; }
    std::size_t address(const char *ptr) const { return ptr - origin_; }

private:
    const char *origin_;
    const char *mem_;
};

} // namespace mmapped

#endif // mapproxy_support_mmapped_memory_hpp_included_
