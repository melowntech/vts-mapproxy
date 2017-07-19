#include <boost/utility/in_place_factory.hpp>
#include <boost/optional.hpp>

#include "utility/buildsys.hpp"
#include "service/cmdline.hpp"

#include "mapproxy/support/mmtileindex.hpp"

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
