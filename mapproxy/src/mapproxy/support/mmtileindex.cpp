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

#include "./mmtileindex.hpp"

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

    // read data size
    bin::read(f, dataSize_);

    {
        // align start of data block
        f.seekg(utility::align(f.tellg(), sizeof(std::uint16_t)));
    }

    // remember memory
    data_ = memory.addr(f.tellg());
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

inline Flag::value_type vts2mm(vts::TileIndex::Flag::value_type in)
{
    Flag::value_type out(0);
    if (in & vts::TileIndex::Flag::mesh) { out |= Flag::data; }
    if (in & vts::TileIndex::Flag::watertight) { out |= Flag::watertight; }
    if (in & vts::TileIndex::Flag::navtile) { out |= Flag::navtile; }

    return out;
}

void QTree::write(std::ostream &f, const vts::QTree &tree)
{
    bin::write(f, MM_QTREE_MAGIC); // 4 bytes
    bin::write(f, std::uint8_t(0)); // reserved
    bin::write(f, std::uint8_t(0)); // reserved

    // order (lod)
    bin::write(f, std::uint8_t(tree.order()));

    // make room for data size
    auto sizePlace(f.tellp());
    f.seekp(sizePlace + std::streampos(sizeof(std::uint32_t)));

    if (tree.empty()) {
        // empty tree, write root maker
        bin::write(f, std::uint8_t(Flag::rootMarker));
    } else if (tree.full()) {
        // only single value, write root marker with value
        bin::write(f, std::uint8_t(Flag::rootMarker | vts2mm(tree.get(0, 0))));
    } else {
        // TODO: write actual tree here
    }

    // compute data size and write to pre-allocated place
    auto end(f.tellp());
    f.seekp(sizePlace);
    std::uint32_t size(end - sizePlace - sizeof(std::uint32_t));
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
