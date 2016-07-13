#ifndef mapproxy_support_python_hpp_included_
#define mapproxy_support_python_hpp_included_

#include <string>
#include <boost/python.hpp>

inline std::string py2utf8(const boost::python::object &s)
{
    return boost::python::extract<std::string>
        (s.attr("encode")("utf-8"));
}

#endif // mapproxy_support_python_hpp_included_
