#include "utility/buildsys.hpp"
#include "service/cmdline.hpp"

#include "mapproxy/support/mmapped/tileindex.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

class TileIndex2MMappedTileIndex : public service::Cmdline {
public:
    TileIndex2MMappedTileIndex()
        : service::Cmdline("mapproxy-ti2mmti", BUILD_TARGET_VERSION)
    {
    }

private:
    void configuration(po::options_description &cmdline
                       , po::options_description &config
                       , po::positional_options_description &pd);

    void configure(const po::variables_map &vars);

    bool help(std::ostream &out, const std::string &what) const;

    int run();

    fs::path input_;
    fs::path output_;
};

void TileIndex2MMappedTileIndex
::configuration(po::options_description &cmdline
                , po::options_description &config
                , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Path to input tile index.")
        ("output", po::value(&output_)->required()
         , "Path to output mmapped tile index.")
        ;

    pd.add("input", 1)
        .add("output", 1);

    (void) config;
}

void TileIndex2MMappedTileIndex::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool TileIndex2MMappedTileIndex::help(std::ostream &out
                                      , const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("mapproxy tileindex to mmapped tileindex converter\n"
                "\n"
                );

        return true;
    }

    return false;
}

int TileIndex2MMappedTileIndex::run()
{
    vts::TileIndex ti;
    ti.load(input_);
    mmapped::TileIndex::write(output_, ti);
    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    return TileIndex2MMappedTileIndex()(argc, argv);
}
