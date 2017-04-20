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

#ifndef mapproxy_support_grid_hpp_included_
#define mapproxy_support_grid_hpp_included_

#include <vector>

#include "math/geometry_core.hpp"

template <typename T>
class Grid {
public:
    typedef T value_type;

    Grid(const math::Size2 &size, const T &fill = T())
        : size_(size)
        , grid_(math::area(size), fill)
    {}

    T& operator()(int x, int y) {
        return grid_[index(x, y)];
    }

    const T& operator()(int x, int y) const {
        return grid_[index(x, y)];
    }

    /** Returns pointer to pixel of non if pixel is invalid
     */
    template <typename Mask>
    const T* operator()(const Mask &mask, int x, int y) const {
        if (!mask(x, y)) { return nullptr; }
        return &grid_[index(x, y)];
    }

private:
    inline int index(int x, int y) const {
#ifndef NDEBUG
        // this is compiled in only in debug mode
        if ((x < 0) || (x >= size_.width)
            || (y < 0) || (y >= size_.height))
        {
            LOGTHROW(err3, Error)
                << "Invalid index [" << x << ", " << y << "] in grid of size "
                << size_ << ". Go and fix your code,";
        }
#endif // NDEBUG
        return y * size_.width + x;
    }

    math::Size2 size_;
    std::vector<T> grid_;
};

#endif // mapproxy_support_grid_hpp_included_
