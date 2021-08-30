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

#include <thread>
#include <condition_variable>
#include <sstream>
#include <deque>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/asio.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/global_fun.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/path.hpp"
#include "utility/gccversion.hpp"
#include "utility/time.hpp"
#include "utility/raise.hpp"

#include "../error.hpp"
#include "../generator.hpp"
#include "../definition.hpp"
#include "./factory.hpp"

namespace fs = boost::filesystem;
namespace asio = boost::asio;
namespace bmi = boost::multi_index;
namespace ba = boost::algorithm;

namespace {

const std::string ResourceFile("resource.json");

} // namespace

void Generator::registerType(const Resource::Generator &type
                             , const Factory::pointer &factory)
{
    Factory::registerType(type, factory);
}

DefinitionBase::pointer Generator::definition(const Resource::Generator &type)
{
    return resource::definition(type);
}

Generator::pointer Generator::create(const Params &params)
{
    try {
        return Factory::findFactory(params.resource.generator)->create(params);
    } catch (const boost::bad_any_cast&) {
        LOGTHROW(err2, InvalidConfiguration)
            << "Passed resource does not match generator <"
            << params.resource.generator << ">.";
    }
    throw;
}

Generator::Generator(const Params &params, const Properties &properties)
    : generatorFinder_(params.generatorFinder), config_(params.config)
    , properties_(properties)
    , resource_(params.resource), savedResource_(params.resource)
    , fresh_(false), system_(params.system)
    , changeEnforced_(false)
    , ready_(false), readySince_(0)
    , demRegistry_(params.demRegistry)
    , replace_(params.replace)
{
    config_.root = (config_.root / resource_.id.referenceFrame
                    / resource_.id.group / resource_.id.id);

    // TODO: handle failed creation
    auto rfile(root() / ResourceFile);

    const auto freshlyCreated(create_directories(root()));

    if (freshlyCreated || !fs::exists(rfile) || system_) {
        // new or forced new resource (system)
        fresh_ = true;
    } else {
        // reopen of existing dataset
        savedResource_ = loadResource(rfile).front();

        // merge both revisions; at least manually changed revision is used
        savedResource_.revision = resource_.revision
            = std::max(resource_.revision, savedResource_.revision);

        const auto freeze(config().freezes(savedResource_.generator.type));

        switch (savedResource_.changed(resource_)) {
        case Changed::withRevisionBump:
            // update revision
            ++resource_.revision;
            changeEnforced_ = true;
            LOG(info3)
                << "Bumped resource <" << resource_.id
                << "> revision to " << resource_.revision
                << " due to definition change.";
            UTILITY_FALLTHROUGH;

        case Changed::no: UTILITY_FALLTHROUGH;
        case Changed::safely:
            // nothing or something non-destructive changed -> re-save
            save(rfile, resource_);
            savedResource_ = resource_;
            break;

        case Changed::yes:
            if (freeze) {
                // different setup, use stored definition
                LOG(warn3)
                    << "Definition of resource <" << resource_.id
                    << "> differs from the one stored in store at "
                    << root() << "; using stored definition.";
                resource_ = savedResource_;

                // force received file class setings even for saved resource
                resource_.fileClassSettings
                    = params.resource.fileClassSettings;
                break;
            } else {
                // changed but not freezing bump revision
                LOG(warn3)
                    << "Definition of resource <" << resource_.id
                    << "> differs from the one stored in store at "
                    << root() << "; bumped revision to "
                    << resource_.revision
                    << " due to disabled resource freezing.";

                ++resource_.revision;
                changeEnforced_ = true;
            }
        }
    }
}

Changed Generator::changed(const Resource &resource) const
{
    switch (auto changed = resource.changed(resource_)) {
    case Changed::yes:
        if (!config().freezes(resource.generator.type)) {
            LOG(warn2)
                << "Definition of resource <" << resource.id
                << "> differs from the one stored in store at "
                << root() << "; using stored definition.";
        }
        return changed;

    default:
        return changed;
    }
}

void Generator::makeReady()
{
    if (fresh_ || changeEnforced_) {
        save(root() / ResourceFile, resource_);
        changeEnforced_ = false;
    }

    ready_ = true;
    readySince_ = utility::usecFromEpoch();

    LOG(info2) << "Ready to serve resource <" << id()
               << "> (type <" << resource().generator << ">).";
}

void Generator::mapConfig(std::ostream &os, ResourceRoot root)
    const
{
    vts::MapConfig mc(mapConfig(root));
    vts::saveMapConfig(mc, os);
}

namespace {

bool isRemote(const std::string &path)
{
    return ((ba::istarts_with(path, "http:")
             || ba::istarts_with(path, "https:")
             || ba::istarts_with(path, "ftp:")));
}

bool isMvt(const std::string &path)
{
    return ba::starts_with(path, "mvt:");
}

} // namespace

std::string Generator::absoluteDataset(const std::string &path)
    const
{
    // handle non-path resources (i.e. URL's)
    if (isMvt(path)) { return "mvt:" + absoluteDataset(path.substr(4)); }
    if (isRemote(path)) { return path; }
    return absolute(path, config_.resourceRoot).string();
}

boost::filesystem::path
Generator::absoluteDataset(const boost::filesystem::path &path)
    const
{
    const auto &spath(path.string());
    // handle non-path resources (i.e. URL's)
    if (isMvt(spath)) { return "mvt:" + absoluteDataset(spath.substr(4)); }
    if (isRemote(path.string())) { return path; }
    return absolute(path, config_.resourceRoot);
}

boost::optional<std::string>
Generator::absoluteDataset(const boost::optional<std::string> &path) const
{
    if (!path) { return path; }
    return absoluteDataset(*path);
}

std::string Generator
::absoluteDataset(const std::string &path
                  , const boost::optional<std::string> &override) const
{
    if (override) { return absoluteDataset(*override); }
    return absoluteDataset(path);
}

boost::optional<boost::filesystem::path>
Generator
::absoluteDatasetRf(const boost::optional<boost::filesystem::path> &path)
    const
{
    if (!path) { return path; }
    return absoluteDataset
        (utility::addExtension(*path, "." + referenceFrameId()));
}

boost::optional<boost::filesystem::path>
Generator
::ignoreNonexistent(const boost::optional<boost::filesystem::path> &path) const
{
    // valid path but file doesn't exist -> invalid
    if (path && !fs::exists(*path)) { return boost::none; }
    // pass parameter
    return path;
}

void Generator::checkReady() const
{
    if (ready_) { return; }
    throw Unavailable("Generator not ready.");
}

std::string Generator::url(GeneratorInterface::Interface iface) const
{
    const auto trailer(prependRoot(fs::path("/"), id()
                                   , GeneratorInterface(type(), iface)
                                   , { ResourceRoot::referenceFrame })
                       .string());

    const auto &prefix(config().externalUrl);

    if (prefix.empty()) { return trailer; }
    if (prefix.back() != '/') { return prefix + "/" + trailer; }
    return prefix + trailer;
}

bool Generator::updatedSince(std::uint64_t timestamp) const
{
    return readySince_ > timestamp;
}

Generator::Task Generator::generateFile(const FileInfo &fileInfo, Sink sink)
    const
{
    if (!properties_.isSupported(fileInfo.interface.interface)) {
        utility::raise<NotFound>
            ("WMTS interface disabled, no <wmts> extension in "
             "reference frame <%s> or not supported by <%s> driver."
             , referenceFrameId(), resource().generator);
    }

    return generateFile_impl(fileInfo, sink);
}

void Generator::updateRevision(unsigned int revision)
{
    resource_.revision = std::max(resource_.revision, revision);
}

void Generator::purge()
{
    boost::system::error_code ec;
    fs::remove_all(root(), ec);
}

