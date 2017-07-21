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
#include "./memory-impl.hpp"

namespace mmapped {

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace {

const char MM_TILEINDEX_MAGIC[4] = { 'M', 'M', 'T', 'I' };

} // namespace

TileIndex::TileIndex(const fs::path &path)
    : memory_(std::make_shared<Memory>(path))
{
    checkHeader(memory_->stream, MM_TILEINDEX_MAGIC, 2, "mmapped tile index");

    // memory stream is positioned just after the header
    const int lods(bin::read<uint8_t>(memory_->stream));

    trees_.reserve(lods);
    for (int lod(0); lod < lods; ++lod) {
        trees_.emplace_back(*memory_);
    }
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

const QTree* TileIndex::tree(vts::Lod lod) const
{
    if (lod >= trees_.size()) { return nullptr; }
    return &trees_[lod];
}

TileIndex::value_type TileIndex::get(const vts::TileId &tileId) const
{
    if (const auto *t = tree(tileId.lod)) {
        return t->get(tileId.x, tileId.y);
    }

    return TileFlag::none;
}

bool TileIndex::validSubtree(vts::Lod lod, const vts::TileId &tileId) const
{
    if (lod >= trees_.size()) { return false; }

    // check all tileindex layers from tileId's lod to the bottom
    for (auto itrees(trees_.begin() + lod), etrees(trees_.end());
         itrees != etrees; ++itrees)
    {
        if (itrees->get(tileId.lod, tileId.x, tileId.y)) { return true; }
    }
    return false;
}

bool TileIndex::validSubtree(const vts::TileId &tileId) const
{
    return validSubtree(tileId.lod, tileId);
}

} // namespace mmapped
