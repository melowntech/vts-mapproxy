#include "./fileclass.hpp"

namespace po = boost::program_options;

void FileClassSettings::from(const boost::any &value)
{
    // TODO: implement me
    (void) value;
}

void FileClassSettings::to(boost::any &value) const
{
    // TODO: implement me
    (void) value;
}

void FileClassSettings::configuration(po::options_description &od
                                      , const std::string &prefix)
{
    auto ao(od.add_options());
    for (auto fc : enumerationValues(FileClass())) {
        if (fc == FileClass::unknown) { continue; }
        auto name(boost::lexical_cast<std::string>(fc));
        ao((prefix + name).c_str()
           , po::value(&maxAges_[static_cast<int>(fc)])
           ->required()->default_value(maxAges_[static_cast<int>(fc)])
           , ("Max age of file class <" + name
              + ">; >=0: Cache-Control: max-age, <0: Cache-Control=no-cache")
           .c_str());
    }
}
