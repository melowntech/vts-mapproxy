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

#include "files.hpp"

#include "generator/files/defaultstyle.json.hpp"
#include "generator/files/wmtsreadme.md.hpp"
#include "generator/files/cesiumreadme.md.hpp"

namespace files {

const vtslibs::storage::SupportFile defaultStyle = {
    defaultstyle_json
    , sizeof(defaultstyle_json)
    , defaultstyle_json_attr_lastModified
    , "application/json; charset=utf-8"
};

const vtslibs::storage::SupportFile wmtsReadme = {
    wmtsreadme_md
    , sizeof(wmtsreadme_md)
    , wmtsreadme_md_attr_lastModified
    , "text/markdown; charset=utf-8"
};

const vtslibs::storage::SupportFile cesiumReadme = {
    cesiumreadme_md
    , sizeof(cesiumreadme_md)
    , cesiumreadme_md_attr_lastModified
    , "text/markdown; charset=utf-8"
};

} // namespace files
