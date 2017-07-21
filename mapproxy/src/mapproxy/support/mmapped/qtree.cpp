#include "dbglog/dbglog.hpp"

#include "utility/filesystem.hpp"
#include "utility/streams.hpp"
#include "utility/binaryio.hpp"
#include "utility/align.hpp"

#include "./tileindex.hpp"
#include "./memory-impl.hpp"

namespace mmapped {

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace {

const char MM_QTREE_MAGIC[4] = { 'M', 'M', 'Q', 'T' };

} // namespace

QTree::QTree(Memory &memory)
    : depth_(), data_(), dataSize_()
{
    auto &f(memory.stream);

    checkHeader(f, MM_QTREE_MAGIC, 2, "mmapped qtree");

    // read tree depth (i.e. lod)
    depth_ = bin::read<std::uint8_t>(f);
    size_ = 1 << depth_;

    // read data size
    dataSize_ = bin::read<std::uint32_t>(f);

    // align start of data block
    std::size_t dataStart(utility::align(f.tellg(), sizeof(std::uint32_t)));

    // remember memory
    data_ = memory.addr(dataStart);

    // skip data
    f.seekg(dataStart + dataSize_);
}

namespace {

inline TileFlag::value_type vts2mm(vts::TileIndex::Flag::value_type in)
{
    return TileFlag::value_type(in);
}

inline TileFlag::value_type vts2mm(const vts::QTree::opt_value_type &in)
{
    return (in ? vts2mm(*in) : TileFlag::value_type(TileFlag::invalid));
}

} // namespace

void QTree::write(std::ostream &f, const vts::QTree &tree)
{
    bin::write(f, MM_QTREE_MAGIC); // 4 bytes
    bin::write(f, std::uint8_t(0)); // reserved
    bin::write(f, std::uint8_t(0)); // reserved

    // order (lod)
    bin::write(f, std::uint8_t(tree.order()));

    // make room for data size
    std::size_t sizePlace(f.tellp());

    // align address after size place
    f.seekp(utility::align(sizePlace + sizeof(std::uint32_t)
                           , sizeof(std::uint32_t)));
    std::size_t dataStart(f.tellp());

    struct Converter {
        Converter(std::ostream &f) : f(f) {}

        void root(vts::QTree::opt_value_type value) {
            // write root value in all nodes
            bin::write(f, vts2mm(value));
            bin::write(f, vts2mm(value));
            bin::write(f, vts2mm(value));
            bin::write(f, vts2mm(value));
        }

        typedef std::array<std::streampos, 4> IndexTable;

        IndexTable children(const vts::QTree::opt_value_type &ul
                            , const vts::QTree::opt_value_type &ur
                            , const vts::QTree::opt_value_type &ll
                            , const vts::QTree::opt_value_type &lr)
        {
            // write values for all 4 children
            bin::write(f, vts2mm(ul));
            bin::write(f, vts2mm(ur));
            bin::write(f, vts2mm(ll));
            bin::write(f, vts2mm(lr));

            // grab current position and prepare table
            std::size_t pos(f.tellp());
            IndexTable table;

            bool first(true);

            auto isIndexable([&](const vts::QTree::opt_value_type &value)
                             -> bool
            {
                // leaf -> non-indexable
                if (value) { return false; }

                // first internal node -> non-indexable
                if (first) {
                    first = false;
                    return false;
                }

                // internal none
                return true;
            });

            auto addIndex([&](const vts::QTree::opt_value_type &value
                              , int index) -> void
            {
                if (isIndexable(value)) {
                    // allocate space for an index in the index table
                    table[index] = pos;
                    pos += sizeof(std::uint32_t);
                } else {
                    // not indexable
                    table[index] = -1;
                }
            });

            addIndex(ul, 0);
            addIndex(ur, 1);
            addIndex(ll, 2);
            addIndex(lr, 3);

            // move past table space and done
            f.seekp(pos);
            return table;
        }

        void enter(const IndexTable &table, int index) {
            const auto indexPos(table[index]);
            if (indexPos < 0) { return; }

            // current file position (here starts the node)
            auto end(f.tellp());

            // rewind to allocated space
            f.seekp(indexPos);

            // calculate jump value (take size of jump value into account)
            std::uint32_t indexValue(end - indexPos - sizeof(indexValue));
            bin::write(f, indexValue);

            // jump to the end again
            f.seekp(end);
        }

        void leave(const IndexTable&, int) {}

        std::ostream &f;
    };

    // convert tree
    {
        Converter converter(f);
        tree.convert(converter);
    }

    // compute data size and write to pre-allocated place
    std::size_t end(f.tellp());
    f.seekp(sizePlace);
    std::uint32_t size(end - dataStart);
    bin::write(f, size);

    // move back to the end
    f.seekp(end);
}

QTree::value_type QTree::get(unsigned int x, unsigned int y) const
{
    if ((x >= size_) || (y >= size_)) { return TileFlag::none; }

    MemoryReader reader(data_);

    // load root value
    const auto &root(reader.read<NodeValue>());

    // shortcut for root node
    if (TileFlag::leaf(root[0])) { return root[0]; }

    // descend
    return get(reader, Node(size_), x, y);
}

QTree::value_type QTree::get(unsigned int depth, unsigned int x
                             , unsigned int y) const
{
    // not trimming -> regular get
    if (depth >= depth_) {
        // too deep, move up and regular get
        return get(x >> (depth - depth_), y >> (depth - depth_));
    }

    // calculate size in of a trimmed tree
    unsigned int size(1 << depth);

    if ((x >= size) || (y >= size)) { return TileFlag::none; }

    MemoryReader reader(data_);

    // load root value
    const auto &root(reader.read<NodeValue>());

    // shortcut for root node
    if (TileFlag::leaf(root[0])) { return root[0]; }

    // shortcut for too shallow virtual tree
    if (size == 1) { return TileFlag::any; }

    // descend
    return get(reader, Node(size), x, y);
}

QTree::value_type QTree::get(MemoryReader &reader, const Node &node
                             , unsigned int x, unsigned int y) const
{
    // load value
    const auto &nodeValue(reader.read<NodeValue>());
    const auto flags(nodeValue.flags());

    // upper-left child
    Node child(node.child());

    if (y < (child.y + child.size)) {
        // upper row
        if (x < (child.x + child.size)) {
            // UL
            if (flags[0]) { return nodeValue[0]; }
            if (child.size == 1) { return TileFlag::any; }

            // jump to the start of node data
            flags.jumpTo(reader, 0);
            return get(reader, child, x, y);
        }

        // UR
        if (flags[1]) { return nodeValue[1]; }
        if (child.size == 1) { return TileFlag::any; }

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
        if (child.size == 1) { return TileFlag::any; }

        // jump to the start of node data
        flags.jumpTo(reader, 2);

        // fix-up child and descend
        child.y += child.size;
        return get(reader, child, x, y);
    }

    // LR
    if (flags[3]) { return nodeValue[3]; }
    if (child.size == 1) { return TileFlag::any; }

    // jump to the start of node data
    flags.jumpTo(reader, 3);

    // fix-up child and descend
    child.x += child.size;
    child.y += child.size;
    return get(reader, child, x, y);
}

} // namespace mmapped
