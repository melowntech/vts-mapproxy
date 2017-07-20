#include <boost/filesystem.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/device/array.hpp>

#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/filesystem.hpp"
#include "utility/streams.hpp"
#include "utility/binaryio.hpp"
#include "utility/align.hpp"

#include "./tileindex.hpp"

namespace mmapped {

namespace fs = boost::filesystem;
namespace bio = boost::iostreams;
namespace bi = boost::interprocess;
namespace bin = utility::binaryio;

namespace {

const char MM_TILEINDEX_MAGIC[4] = { 'M', 'M', 'T', 'I' };
const char MM_QTREE_MAGIC[4] = { 'M', 'M', 'Q', 'T' };

template <int magicSize>
void checkHeader(std::istream &f
                 , const char (&expectMagic)[magicSize]
                 , std::size_t extraBytes
                 , const char *what)
{
    char magic[magicSize];
    bin::read(f, magic);
    if (std::memcmp(magic, expectMagic, magicSize)) {
        LOGTHROW(err2, std::runtime_error)
            << "Mapped region is not a " << what << ".";
    }

    // skip extra header bytes
    f.seekg(extraBytes, std::ios_base::cur);
}

} // namespace

struct Memory {
    Memory(const boost::filesystem::path &path)
        : size(utility::fileSize(path))
        , file(path.c_str(), bi::read_only)
        , region(file, bi::read_only, 0, size)
        , data(static_cast<char*>(region.get_address()))
        , buffer(data, data + size)
        , stream(&buffer)
    {
        checkHeader(stream, MM_TILEINDEX_MAGIC, 2, "mmapped tile index");
    }

    const char* addr(std::size_t pos) const { return data + pos; }

    std::size_t size;
    bi::file_mapping file;
    bi::mapped_region region;
    const char *data;

    bio::stream_buffer<bio::array_source> buffer;
    std::istream stream;
};

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

TileIndex::TileIndex(const fs::path &path)
    : memory_(std::make_shared<Memory>(path))
{
    // memory stream is positioned just after the header
    const int lods(bin::read<uint8_t>(memory_->stream));

    trees_.reserve(lods);
    for (int lod(0); lod < lods; ++lod) {
        trees_.emplace_back(*memory_);
    }
}

inline TileFlag::value_type vts2mm(vts::TileIndex::Flag::value_type in)
{
    return TileFlag::value_type(in);
}

inline TileFlag::value_type vts2mm(const vts::QTree::opt_value_type &in)
{
    return (in ? vts2mm(*in) : TileFlag::value_type(TileFlag::invalid));
}

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

void TileIndex::write(std::ostream &f, const vts::TileIndex &ti)
{
    bin::write(f, MM_TILEINDEX_MAGIC); // 4 bytes
    bin::write(f, std::uint8_t(0)); // reserved
    bin::write(f, std::uint8_t(0)); // reserved

    if (ti.empty()) { return; }

    // lod count (max lod + 1)
    const auto lodCount(ti.maxLod() + 1);
    bin::write(f, std::uint8_t(lodCount));

    // write all trees
    for (vts::Lod lod(0); lod < lodCount; ++lod) {
        if (const auto *tree = ti.tree(lod)) {
            // tree exists, write
            QTree::write(f, *tree);
        } else {
            // tree doesn't exist, write empty
            QTree::write(f, vts::QTree(lod));
        }
    }
}

void TileIndex::write(const boost::filesystem::path &path
                      , const vts::TileIndex &ti)
{
    utility::ofstreambuf f;
    f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    f.open(path.string(), std::ifstream::out | std::ifstream::trunc);

    write(f, ti);

    f.close();
}

} // namespace mmapped
