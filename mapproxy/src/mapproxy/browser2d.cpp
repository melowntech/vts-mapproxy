#include "./browser2d.hpp"

#include "browser2d/index.html.hpp"
#include "browser2d/leaflet.js.hpp"
#include "browser2d/leaflet.css.hpp"

namespace browser2d {

const vadstena::storage::SupportFile::Files supportFiles =
{
    { "index.html"
      , {
            index_html
            , sizeof(index_html)
            , index_html_attr_lastModified
            , "text/html; charset=utf-8"
        }
    }, { "leaflet.js"
         , {
            leaflet_js
            , sizeof(leaflet_js)
            , leaflet_js_attr_lastModified
            , "application/javascript;  charset=utf-8"
        }
    }, { "leaflet.css"
         , {
            leaflet_css
            , sizeof(leaflet_css)
            , leaflet_css_attr_lastModified
            , "text/css; charset=utf-8"
        }
    }
};

} // namespace browser2d
