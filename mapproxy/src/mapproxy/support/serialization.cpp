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

#include "./python.hpp"
#include "./serialization.hpp"

Resource::Id::list introspectionListFrom(const Json::Value &introspection
                                         , const std::string &key)
{
    if (!introspection.isMember(key)) { return {}; }

    const auto &value(introspection[key]);
    auto getRid([](const Json::Value &item) -> Resource::Id
    {
        Resource::Id rid;
        Json::get(rid.group, item, "group");
        Json::get(rid.id, item, "id");
        return rid;
    });

    Resource::Id::list out;

    if (value.isArray()) {
        for (const auto &item : value) {
            out.push_back(getRid(item));
        }
    } else {
        out.push_back(getRid(value));
    }

    return out;
}

void introspectionListTo(Json::Value &introspection
                           , const std::string &key
                           , const Resource::Id::list &resources)
{
    if (resources.empty()) { return; }

    auto &list(introspection[key] = Json::arrayValue);
    for (const auto &rid : resources) {
        auto &item(list.append(Json::objectValue));
        item["group"] = rid.group;
        item["id"] = rid.id;
    }
}

Resource::Id::list
introspectionListFrom(const boost::python::dict &introspection
                      , const std::string &key)
{
    if (!introspection.has_key(key)) { return {}; }

    auto getRid([](const boost::python::dict &item) -> Resource::Id
    {
        Resource::Id rid;
        rid.group = py2utf8(item["group"]);
        rid.id = py2utf8(item["id"]);
        return rid;
    });

    Resource::Id::list out;

    // TODO: support list
    boost::python::dict value(introspection[key]);
    out.push_back(getRid(value));

    return out;
}

boost::optional<Resource::Id>
introspectionIdFrom(const Json::Value &introspection, const std::string &key)
{
    if (!introspection.isMember(key)) { return {}; }

    const auto &value(introspection[key]);
    auto getRid([](const Json::Value &item) -> Resource::Id
    {
        Resource::Id rid;
        Json::get(rid.group, item, "group");
        Json::get(rid.id, item, "id");
        return rid;
    });

    return getRid(value);
}

void introspectionIdTo(Json::Value &introspection
                       , const std::string &key
                       , const boost::optional<Resource::Id> &resource)
{
    if (!resource) { return; }

    auto &item(introspection[key] = Json::objectValue);
    item["group"] = resource->group;
    item["id"] = resource->id;
}

boost::optional<Resource::Id>
introspectionIdFrom(const boost::python::dict &introspection
                    , const std::string &key)
{
    if (!introspection.has_key(key)) { return {}; }

    auto getRid([](const boost::python::dict &item) -> Resource::Id
    {
        Resource::Id rid;
        rid.group = py2utf8(item["group"]);
        rid.id = py2utf8(item["id"]);
        return rid;
    });

    boost::python::dict value(introspection[key]);
    return getRid(value);
}
