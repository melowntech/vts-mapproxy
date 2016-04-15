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
            , "text/html"
        }
    }, { "leaflet.js"
         , {
            leaflet_js
            , sizeof(leaflet_js)
            , leaflet_js_attr_lastModified
            , "text/html"
        }
    }, { "leaflet.css"
         , {
            leaflet_css
            , sizeof(leaflet_css)
            , leaflet_css_attr_lastModified
            , "text/css"
        }
    }
};

} // namespace browser2d
