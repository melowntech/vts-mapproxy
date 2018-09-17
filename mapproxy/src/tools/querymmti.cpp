/**
 * Copyright (c) 2018 Melown Technologies SE
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

#include <boost/utility/in_place_factory.hpp>
#include <boost/optional.hpp>

#include "utility/buildsys.hpp"
#include "service/cmdline.hpp"

#include "mapproxy/support/mmapped/tileindex.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

class QueryMappedTileIndex : public service::Cmdline {
public:
    QueryMappedTileIndex()
        : service::Cmdline("mapproxy-querymmti", BUILD_TARGET_VERSION)
    {
    }

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    fs::path ti_;
    vts::TileId tileId_;
};

void QueryMappedTileIndex
::configuration(po::options_description &cmdline
                , po::options_description &config
                , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("tileIndex", po::value(&ti_)->required()
         , "Path to tile index.")
        ("tileId", po::value(&tileId_)->required()
         , "Tile ID to query.")
        ;

    pd.add("tileIndex", 1)
        .add("tileId", 1);

    (void) config;
}

void QueryMappedTileIndex::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool QueryMappedTileIndex::help(std::ostream &out
                                      , const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("mapproxy mmapped tileindex query tool\n"
                "\n"
                );

        return true;
    }

    return false;
}

int QueryMappedTileIndex::run()
{
    // try to open
    boost::optional<mmapped::TileIndex> mti;
    boost::optional<vts::TileIndex> ti;

    try {
        mti = boost::in_place(ti_);
    } catch (std::exception) {
        ti = boost::in_place();
        ti->load(ti_);
    }

    if (mti) {
        std::cout << std::bitset<8>(mti->get(tileId_)) << std::endl;
    } else {
        std::cout << std::bitset<32>(ti->get(tileId_)) << std::endl;
    }

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    return QueryMappedTileIndex()(argc, argv);
}
