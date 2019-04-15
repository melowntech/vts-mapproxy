/**
 * Copyright (c) 2019 Melown Technologies SE
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

#include "cesium.hpp"

#include "cesium/cesium.html.hpp"
#include "cesium/cesium.js.hpp"
#include "cesium/melown-technologies-logo-28.png.hpp"

namespace cesium {

const vtslibs::storage::SupportFile::Files supportFiles =
{
    { "browser.html"
      , {
            cesium_html
            , sizeof(cesium_html)
            , cesium_html_attr_lastModified
            , "text/html; charset=utf-8"
        }
    }, { "cesium.js"
      , {
            cesium_js
            , sizeof(cesium_js)
            , cesium_js_attr_lastModified
            , "application/javascript; charset=utf-8"
        }
    }, { "melowntech.png"
      , {
            melowntech_png
            , sizeof(melowntech_png)
            , melowntech_png_attr_lastModified
            , "image/png"
        }
    }
};

} // namespace cesium
