#include "./browser2d.hpp"

#include "browser2d/index.html.hpp"
#include "browser2d/index.js.hpp"

namespace browser2d {

const vtslibs::storage::SupportFile::Files supportFiles =
{
    { "index.html"
      , {
            index_html
            , sizeof(index_html)
            , index_html_attr_lastModified
            , "text/html; charset=utf-8"
        }
    }, { "index.js"
         , {
            index_js
            , sizeof(index_js)
            , index_js_attr_lastModified
            , "application/javascript; charset=utf-8"
        }
    }
};

} // namespace browser2d
