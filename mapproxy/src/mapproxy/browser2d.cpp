#include "./browser2d.hpp"

#include "browser2d/index.html.hpp"

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
    }};

} // namespace browser2d
