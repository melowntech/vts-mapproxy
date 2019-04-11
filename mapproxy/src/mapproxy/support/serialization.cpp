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

#include "jsoncpp/as.hpp"

#include "python.hpp"
#include "introspection.hpp"

namespace introspection {

Layers layersFrom(const Json::Value &introspection, const std::string &key)
{
    if (!introspection.isMember(key)) { return {}; }

    const auto &value(introspection[key]);

    const auto &local([](const Json::Value &item) -> LocalLayer
    {
        LocalLayer l;
        Json::get(l.group, item, "group");
        Json::get(l.id, item, "id");
        return l;
    });

    const auto &remote([](const Json::Value &item) -> RemoteLayer
    {
        RemoteLayer l;
        Json::get(l.id, item, "id");
        Json::get(l.url, item, "url");
        return l;
    });

    Layers out;

    const auto &add([&](const Json::Value &item)
    {
        if (item.isMember("url")) {
            out.push_back(remote(item));
        } else {
            out.push_back(local(item));
        }
    });

    if (value.isArray()) {
        for (const auto &item : value) {
            add(item);
        }
    } else {
        add(value);
    }

    return out;
}

void layersTo(Json::Value &introspection, const std::string &key
              , const Layers &layers)
{
    if (layers.empty()) { return; }

    struct Visitor : boost::static_visitor<void> {
        Json::Value &list;
        Visitor(Json::Value &list) : list(list) {}

        void operator()(const LocalLayer &l) {
            auto &item(list.append(Json::objectValue));
            item["group"] = l.group;
            item["id"] = l.id;
        }

        void operator()(const RemoteLayer &l) {
            auto &item(list.append(Json::objectValue));
            item["id"] = l.id;
            item["url"] = l.url;
        }
    } visitor(introspection[key] = Json::arrayValue);

    for (const auto &l : layers) { boost::apply_visitor(visitor, l); }
}

Layers layersFrom(const boost::python::dict &introspection
                  , const std::string &key)
{
    if (!introspection.has_key(key)) { return {}; }

    const auto &local([](const boost::python::dict &item) -> LocalLayer
    {
        LocalLayer l;
        l.group = py2utf8(item["group"]);
        l.id = py2utf8(item["id"]);
        return l;
    });

    const auto &remote([](const boost::python::dict &item) -> RemoteLayer
    {
        RemoteLayer l;
        l.id = py2utf8(item["id"]);
        l.url = py2utf8(item["url"]);
        return l;
    });

    Layers out;

    const auto &add([&](const boost::python::dict &item)
    {
        if (item.has_key("url")) {
            out.push_back(remote(item));
        } else {
            out.push_back(local(item));
        }
    });

    // TODO: support list
    boost::python::dict value(introspection[key]);
    add(value);
    return out;
}

Resource::OptId idFrom(const Json::Value &introspection
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

    return getRid(value);
}

void idTo(Json::Value &introspection, const std::string &key
          , const Resource::OptId &resource)
{
    if (!resource) { return; }

    auto &item(introspection[key] = Json::objectValue);
    item["group"] = resource->group;
    item["id"] = resource->id;
}

Resource::OptId idFrom(const boost::python::dict &introspection
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

} // namespace introspection
