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

#ifndef mapproxy_support_mmtileindex_hpp_included_
#define mapproxy_support_mmtileindex_hpp_included_

#include <iostream>

#include "vts-libs/vts/tileindex.hpp"
#include "vts-libs/vts/tileset/tilesetindex.hpp"

// #define MMAPTI_DEBUG

#ifdef MMAPTI_DEBUG
#include "dbglog/dbglog.hpp"
#endif

namespace vts = vtslibs::vts;

/** Simplified tileindex that employs memory mapped quadtrees.
 *
 *  Used to save precious memory.
 *
 *  Handles only 3 flags: mesh, watertight and navtile flags, space for 5 more
 *  flags is available.
 *
 *  Non-leaf nodes are marked by invalid combination (mesh=false,
 *  watertight=true)
 */
namespace mmapped {

/** Memory information.
 */
class Memory;

class MemoryReader {
public:
    MemoryReader(const char *mem) : origin_(mem), mem_(mem) {}

    template <typename T> const T& read() {
        const T *ptr(reinterpret_cast<const T*>(mem_));
#ifdef MMAPTI_DEBUG
        LOG(info4) << "Reading from " << (void*)(ptr)
                   << " (index: " << index(mem_) << ")";
#endif
        mem_ += sizeof(T);
        return *ptr;
    }

    template <typename T> void skip() {
#ifdef MMAPTI_DEBUG
        LOG(info4) << "Skipping " << sizeof(T) << " bytes from "
                   << (void*)(mem_) << " (index: " << index(mem_) << ")";
#endif
        mem_ += sizeof(T);
    }

    // read and apply jump value
    template <typename T> void jump() {
        const auto value(read<T>());
#ifdef MMAPTI_DEBUG
        LOG(info4) << "Jumped from " << (void*)(mem_)
                   << " (index: " << index(mem_) << ")"
                   << " by " << value
                   << " to " << (void*)(mem_ + value)
                   << " (index: " << index(mem_ + value) << ")";
#endif
        mem_ += value;
    }

    // Conditional jump
    template <typename T> void jump(bool proceed) {
        if (proceed) { jump<T>(); }
    }

    // Conditional jump chain
    template <typename T, typename ...Args>
    void jump(bool proceed, Args &&...args) {
        jump<T>(proceed);
        jump<T>(std::forward<Args>(args)...);
    }

private:
    std::size_t index(const char *ptr) const { return ptr - origin_; }

    const char *origin_;
    const char *mem_;
};

struct Flag {
    typedef std::uint8_t value_type;

    enum : value_type {
        mesh = vts::TileIndex::Flag::mesh // mesh (or some other data) present
        , watertight = vts::TileIndex::Flag::watertight // no holes in data
        , navtile = vts::TileIndex::Flag::navtile // navile present

        , data = mesh // alias for mesh
        , none = 0x00 // nothing set
        , any = 0xff // anything set
        /** invalid value used for internal tree nodes
         */
        , invalid = any & ~watertight
    };

    static bool leaf(value_type value) { return value != invalid; }
    static bool internal(value_type value) { return value == invalid; }
};

class QTree {
public:
    typedef std::vector<QTree> list;
    typedef Flag::value_type value_type;

    /** Loads QTree from memory at current memory position.
     */
    QTree(Memory &memory);

    value_type get(unsigned int x, unsigned int y) const;

    static void write(std::ostream &out, const vts::QTree &tree);

    struct Node {
        unsigned int size;
        unsigned int depth;
        unsigned int x;
        unsigned int y;

        Node(unsigned int size, unsigned int depth = 0, unsigned int x = 0
             , unsigned int y = 0)
            : size(size), depth(depth), x(x), y(y)
        {}

        Node child(unsigned int ix = 0, unsigned int iy = 0) const {
            return { size >> 1, depth + 1, x + ix, y + iy };
        }

        void shift(int diff) {
            if (diff >= 0) {
                size >>= diff;
                x >>= diff;
                y >>= diff;
            } else {
                size <<= -diff;
                x <<= -diff;
                y <<= -diff;
            }
        }
    };

    struct NodeValue {
        value_type value[4];
        value_type operator[](std::size_t index) const { return value[index]; }
    };

private:
    value_type get(MemoryReader &reader, const Node &node
                   , unsigned int x, unsigned int y) const;

    unsigned int depth_;
    unsigned int size_;
    const char *data_;
    std::size_t dataSize_;
};

class TileIndex {
public:
    typedef Flag::value_type value_type;

    TileIndex(const boost::filesystem::path &path);

    value_type get(const vts::TileId &tileId) const;

    /** Save vts TileIndex into this mmapped tile index.
     */
    static void write(std::ostream &out, const vts::TileIndex &ti);

    static void write(const boost::filesystem::path &path
                      , const vts::TileIndex &ti);

private:
    const QTree* tree(vts::Lod lod) const;

    std::shared_ptr<Memory> memory_;
    QTree::list trees_;
};

// inlines

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const QTree::Node &n)
{
    return os << "Node(depth=" << n.depth << ", size=" << n.size
              << ", x=" << n.x << ", y=" << n.y << ")"
        ;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const QTree::NodeValue &nv)
{
    os << "NodeValue(";
    if (Flag::internal(nv[0])) { os << "*"; }
    else { os << std::bitset<8>(nv[0]); }
    os << ", ";
    if (Flag::internal(nv[1])) { os << "*"; }
    else { os << std::bitset<8>(nv[1]); }
    os << ", ";
    if (Flag::internal(nv[2])) { os << "*"; }
    else { os << std::bitset<8>(nv[2]); }
    os << ", ";
    if (Flag::internal(nv[3])) { os << "*"; }
    else { os << std::bitset<8>(nv[3]); }
        ;
    return os << ")";
}

inline const QTree* TileIndex::tree(vts::Lod lod) const
{
    if (lod >= trees_.size()) { return nullptr; }
    return &trees_[lod];
}

inline TileIndex::value_type TileIndex::get(const vts::TileId &tileId) const
{
    if (const auto *t = tree(tileId.lod)) {
        return t->get(tileId.x, tileId.y);
    }

    return Flag::none;
}

inline QTree::value_type QTree::get(unsigned int x, unsigned int y) const
{
    if ((x >= size_) || (y >= size_)) { return Flag::none; }

    MemoryReader reader(data_);

    // load root value
    const auto &root(reader.read<NodeValue>());

// #ifdef MMAPTI_DEBUG
//     LOG(info4) << "Root: " << std::bitset<8>(root[0]);
// #endif

    // shortcut for node
    if (Flag::leaf(root[0])) { return root[0]; }

    // descend
    return get(reader, Node(size_), x, y);
}

inline QTree::value_type QTree::get(MemoryReader &reader, const Node &node
                                    , unsigned int x, unsigned int y) const
{
    // load value
    const auto &nodeValue(reader.read<NodeValue>());

    // upper-left child
    Node child(node.child());

// #ifdef MMAPTI_DEBUG
//     LOG(info4) << "node=" << node << "; "
//                << "ul=" << child << "; "
//                << nodeValue
//                << "; x=" << x << ", y=" << y << ".";
// #endif

    if (y < (child.y + child.size)) {
        // upper row
        if (x < (child.x + child.size)) {
            // UL
            if (Flag::leaf(nodeValue[0])) { return nodeValue[0]; }

            // skip over jump value and descend
            reader.skip<std::uint32_t>();
            return get(reader, child, x, y);
        }

        // jump over UL node
        reader.jump<std::uint32_t>(Flag::internal(nodeValue[0]));

        // UR
        if (Flag::leaf(nodeValue[1])) { return nodeValue[1]; }

        // skip over jump value, fix-up child and descend
        reader.skip<std::uint32_t>();
        child.x += child.size;
        return get(reader, child, x, y);
    }

    // lower row

    // jump over UL and UR nodes
    reader.jump<std::uint32_t>(Flag::internal(nodeValue[0])
                               , Flag::internal(nodeValue[1]));

    if (x < (child.x + child.size)) {
        // LL
        if (Flag::leaf(nodeValue[2])) { return nodeValue[2]; }

        // skip over jump value, fix-up child and descend
        reader.skip<std::uint32_t>();
        child.y += child.size;
        return get(reader, child, x, y);
    }

    // skip over LL node
    reader.jump<std::uint32_t>(Flag::internal(nodeValue[2]));

    // LR
    if (Flag::leaf(nodeValue[3])) { return nodeValue[3]; }

    // skip over jump value, fix-up child and descend
    reader.skip<std::uint32_t>();
    child.x += child.size;
    child.y += child.size;
    return get(reader, child, x, y);
}

} // namespace mmapped

#endif // mapproxy_support_tileindex_hpp_included_
