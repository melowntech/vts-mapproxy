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

#ifndef mapproxy_support_mmapped_qtree_hpp_included_
#define mapproxy_support_mmapped_qtree_hpp_included_

#include <array>
#include <iostream>

#include "dbglog/dbglog.hpp"

#include "./tileflags.hpp"
#include "./memory.hpp"

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

class QTree {
public:
    typedef std::vector<QTree> list;
    typedef TileFlag::value_type value_type;

    /** Loads QTree from memory at current memory position.
     */
    QTree(Memory &memory);

    value_type get(unsigned int x, unsigned int y) const;

    static void write(std::ostream &out, const vts::QTree &tree);

    enum class Filter {
        black, white, both
    };

    math::Size2 size() const { return math::Size2(size_, size_); }

    /** Runs op(x, y, xsize, value) for each node based on filter.
     */
    template <typename Op>
    void forEachNode(const Op &op, Filter filter = Filter::both) const;

    /** Runs op(x, y, value) for each node based on filter in subtree starting
     *  at given index.
     */
    template <typename Op>
    void forEachNode(unsigned int depth, unsigned int x, unsigned int y
                     , const Op &op, Filter filter = Filter::both) const;

    typedef math::Extents2_<unsigned int> Extents;

private:
    struct Node;
    struct NodeValue;

    value_type get(MemoryReader &reader, const Node &node
                   , unsigned int x, unsigned int y) const;

    /** Called from forEachQuad */
    template <typename Op>
    void descend(MemoryReader &reader, const Node &node
                 , const Op &op, Filter filter
                 , const Extents *extents) const;

    unsigned int depth_;
    unsigned int size_;
    const char *data_;
    std::size_t dataSize_;
};

struct QTree::Node {
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

    template <typename Op>
    void call(const Op &op, Filter filter, value_type value) const;

    bool checkExtents(const Extents *&extents) const;
};

struct QTree::NodeValue {
    value_type value[4];

    value_type operator[](std::size_t index) const { return value[index]; }

    bool leaf(std::size_t index) const { return TileFlag::leaf(value[index]); }
    bool internal(std::size_t index) const {
        return TileFlag::internal(value[index]);
    }

    struct Flags {
        std::array<bool, 4> flags;
        int leafCount;
        int internalCount;
        int jumpableLimit;
        int firstInternalNode;

        Flags(const NodeValue &nv)
            : flags{{ nv.leaf(0), nv.leaf(1), nv.leaf(2), nv.leaf(3) }}
            , leafCount(flags[0] + flags[1] + flags[2] + flags[3])
            , internalCount(4 - leafCount)
            , jumpableLimit(internalCount - 1)
            , firstInternalNode()
        {
            for (; firstInternalNode < 4; ++firstInternalNode) {
                if (!flags[firstInternalNode]) { break; }
            }
            // firstInternalNode is set to 4 -> no internal node
        }

        bool operator[](std::size_t index) const { return flags[index]; }

        bool jumpable(int index) const {
            return ((index < jumpableLimit) && !flags[index]);
        }

        /** Jumps to node referenced by node's index.
         *
         *  Causes error if node is not internal node
         *
         * This function assumes that reader is positioned at the start of
         * the index table
         */
        void jumpTo(MemoryReader &reader, int node) const {
            if (node == firstInternalNode) {
                // skip whole table and land at the first internal node's
                // data
                reader.skip<std::uint32_t>(internalCount - 1);
                return;
            }

            // skip intermediate internal nodes
            for (int i(firstInternalNode + 1); i < node; ++i) {
                reader.skip<std::uint32_t>(!flags[i]);
            }

            if (flags[node]) {
                LOGTHROW(err2, std::runtime_error)
                    << "Node " << node << " is not an internal node "
                    << " at address " << reader.address() << ".";
            }

            // and jump to node only if not the first one
            reader.jump<std::uint32_t>();
        }

        /** Loads index table for all 4 nodes. All indices are fixed to absolute
         * address from memory origin.
         *
         * This function assumes that reader is positioned at the start of
         * the index table
         */
        std::array<std::uint32_t, 4> indexTable(MemoryReader &reader) const {
            std::array<std::uint32_t, 4> it{{ 0, 0, 0, 0 }};

            // done if no internal node
            if (!internalCount) { return it; }

            // skip first internal node and load explicit table
            for (int node(firstInternalNode + 1); node < 4; ++node) {
                if (flags[node]) { continue; }

                // load absolute jump address
                it[node] = reader.jumpAddress<std::uint32_t>();
            }

            // we stand at the start of the first internal node, record current
            // for this node
            it[firstInternalNode] = reader.address();

            // done
            return it;
        }
    };

    Flags flags() const { return Flags(*this); }
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
    if (TileFlag::internal(nv[0])) { os << "*"; }
    else { os << std::bitset<8>(nv[0]); }
    os << ", ";
    if (TileFlag::internal(nv[1])) { os << "*"; }
    else { os << std::bitset<8>(nv[1]); }
    os << ", ";
    if (TileFlag::internal(nv[2])) { os << "*"; }
    else { os << std::bitset<8>(nv[2]); }
    os << ", ";
    if (TileFlag::internal(nv[3])) { os << "*"; }
    else { os << std::bitset<8>(nv[3]); }
    return os << ")";
}

template <typename Op>
void QTree::Node::call(const Op &op, Filter filter, value_type value) const
{
    switch (filter) {
    case Filter::black: if (value) { return; }; break;
    case Filter::white: if (!value) { return; }; break;
    default: break;
    }

    op(x, y, size, value);
}

inline bool QTree::Node::checkExtents(const Extents *&extents) const
{
    if (!extents) { return true; }

    if ((x + size) <= extents->ll(0)) { return false; }
    if (x >= extents->ur(0)) { return false; }
    if ((y + size) <= extents->ll(1)) { return false; }
    if (y >= extents->ur(1)) { return false; }

    return true;
}

inline QTree::value_type QTree::get(unsigned int x, unsigned int y) const
{
    if ((x >= size_) || (y >= size_)) { return TileFlag::none; }

    MemoryReader reader(data_);

    // load root value
    const auto &root(reader.read<NodeValue>());

// #ifdef MMAPTI_DEBUG
//     LOG(info4) << "Root: " << std::bitset<8>(root[0]);
// #endif

    // shortcut for node
    if (TileFlag::leaf(root[0])) { return root[0]; }

    // descend
    return get(reader, Node(size_), x, y);
}

inline QTree::value_type QTree::get(MemoryReader &reader, const Node &node
                                    , unsigned int x, unsigned int y) const
{
    // load value
    const auto &nodeValue(reader.read<NodeValue>());
    const auto flags(nodeValue.flags());

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
            if (flags[0]) { return nodeValue[0]; }

            // jump to the start of node data
            flags.jumpTo(reader, 0);
            return get(reader, child, x, y);
        }

        // UR
        if (flags[1]) { return nodeValue[1]; }

        // jump to the start of node data
        flags.jumpTo(reader, 1);

        // fix-up child and descend
        child.x += child.size;
        return get(reader, child, x, y);
    }

    // lower row

    if (x < (child.x + child.size)) {
        // LL
        if (flags[2]) { return nodeValue[2]; }

        // jump to the start of node data
        flags.jumpTo(reader, 2);

        // fix-up child and descend
        child.y += child.size;
        return get(reader, child, x, y);
    }

    // LR
    if (flags[3]) { return nodeValue[3]; }

    // jump to the start of node data
    flags.jumpTo(reader, 3);

    // fix-up child and descend
    child.x += child.size;
    child.y += child.size;
    return get(reader, child, x, y);
}

template <typename Op>
void QTree::forEachNode(const Op &op, Filter filter) const
{
    MemoryReader reader(data_);

    // load root value
    const auto &root(reader.read<NodeValue>());

    // shortcut for root node
    if (TileFlag::leaf(root[0])) {
        Node(size_).call(op, filter, root[0]);
        return;
    }

    // descend
    descend(reader, Node(size_), op, filter, nullptr);
}

template <typename Op>
void QTree::forEachNode(unsigned int depth, unsigned int x, unsigned int y
                        , const Op &op, Filter filter) const
{
    if ((x >= size_) || (y >= size_)) {
        // inside?
        return Node(size_).call(op, filter, TileFlag::none);
    }

    MemoryReader reader(data_);

    // load root value
    const auto &root(reader.read<NodeValue>());

    // shortcut for root node
    if (TileFlag::leaf(root[0])) {
        return Node(size_).call(op, filter, root[0]);
    }

    // fix depth
    if (depth > depth_) {
        const auto diff(depth - depth_);
        depth = depth_;
        x >>= diff;
        y >>= diff;
    }

    // compute extents
    const auto diff(depth_ - depth);
    const auto size(size_ >> diff);
    Extents::point_type ll(x << depth, y << depth);
    Extents::point_type ur(ll(0) + size, ll(1) + size);
    Extents extents(ll, ur);

    // and descend
    descend(reader, Node(size_), op, filter, &extents);
}

template <typename Op>
void QTree::descend(MemoryReader &reader, const Node &node
                    , const Op &op, Filter filter
                    , const Extents *extents) const
{
    // load value
    const auto &nodeValue(reader.read<NodeValue>());
    const auto flags(nodeValue.flags());
    const auto indexTable(flags.indexTable(reader));

    auto processSubtree([&](int i, const Node &node) -> void
    {
        if (flags[i]) {
            // leaf
            return node.call(op, filter, nodeValue[i]);
        }

        // internal node

        // terminate descent if out of extents
        if (!node.checkExtents(extents)) { return; }

        // inside valid area
        reader.seek(indexTable[i]);
        descend(reader, node, op, filter, extents);
    });

    // descend to subtrees
    const auto ul(node.child());
    processSubtree(0, ul);
    processSubtree(1, node.child(ul.size));
    processSubtree(2, node.child(0, ul.size));
    processSubtree(2, node.child(ul.size, ul.size));
}

} // namespace mmapped

#endif // mapproxy_support_mmapped_qtree_hpp_included_
