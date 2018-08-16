/**
 * Copyright (c) 2018 Melown Technologies SE
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

#ifndef mapproxy_mapproxy_hpp_included_
#define mapproxy_mapproxy_hpp_included_

#include <cstdint>

#include <boost/filesystem/path.hpp>

#include "service/ctrlclient.hpp"

#include "./resource.hpp"

/** This is not mapproxy server but client to query running mapproxy.
 *  Mapproxy itself is in main.cpp.
 */
class Mapproxy {
public:
    Mapproxy(const boost::filesystem::path &ctrl)
        : ctrl_(ctrl)
    {}

    /** Is given reference frame supported?
     */
    bool supportsReferenceFrame(const std::string &referenceFrame);

    /** Is the given resource known by mapproxy.
     */
    bool has(const Resource::Id &resourceId) const;

    /** Is given resource ready?
     */
    bool isReady(const Resource::Id &resourceId) const;

    /** Compose URL this resource is available at.
     */
    std::string url(const Resource::Id &resourceId) const;

    /** Update resources.
     *  \return timestamp in usec
     */
    std::uint64_t updateResources();

    /** Was list of known resources updated since given timestamp (in usec).
     */
    bool updatedSince(std::uint64_t timestamp) const;

    /** Was given resource updated. Resource is updated only when new or differs
     *  from different version.
     */
    bool updatedSince(const Resource::Id &resourceId, std::uint64_t timestamp
                      , bool nothrow = false)
        const;

private:
    // mutable because we want to mark non-changing functions as const
    mutable service::CtrlClient ctrl_;
};


#endif // mapproxy_mapproxy_hpp_included_
