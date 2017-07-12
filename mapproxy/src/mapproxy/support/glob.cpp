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

#include <glob.h>

#include <cstring>

#include "dbglog/dbglog.hpp"

#include "./glob.hpp"

std::vector<boost::filesystem::path>
globPath(const boost::filesystem::path &pattern)
{
    struct Glob {
        ::glob_t g;

        Glob() { std::memset(&g, 0, sizeof(g)); }
        ~Glob() { ::globfree(&g); }

        typedef char** const_iterator;

        const_iterator begin() const { return g.gl_pathv; }
        const_iterator end() const { return g.gl_pathv + g.gl_pathc; }
    } g;

    const int flags(GLOB_ERR | GLOB_MARK | GLOB_BRACE | GLOB_NOMAGIC);

    const auto res(::glob(pattern.c_str(), flags, nullptr, &g.g));

    if (res) {
        switch (res) {
        case GLOB_NOSPACE:
            LOGTHROW(err1, GlobError) << "Glob: out of memory.";
            break;

        case GLOB_ABORTED:
            LOGTHROW(err1, GlobError) << "Glob: read error.";
            break;

        case GLOB_NOMATCH:
            return {};

        default:
            LOGTHROW(err1, GlobError)
                << "Glob: unknown return status " << res << ".";
        }
    }

    // copy result
    std::vector<boost::filesystem::path> paths;

    // we have some data here
    for (const auto &path : g) {
        paths.push_back(path);
    }

    return paths;
}

