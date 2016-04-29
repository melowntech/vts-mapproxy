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
