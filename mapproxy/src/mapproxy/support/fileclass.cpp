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
