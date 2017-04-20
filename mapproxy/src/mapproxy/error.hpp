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

#ifndef mapproxy_error_hpp_included_
#define mapproxy_error_hpp_included_

#include <stdexcept>
#include <string>

#include "http/error.hpp"

struct Error : std::runtime_error {
    Error(const std::string &message) : std::runtime_error(message) {}
};

struct UnknownResourceBackend : Error {
    UnknownResourceBackend(const std::string &message) : Error(message) {}
};

struct UnknownGenerator : Error {
    UnknownGenerator(const std::string &message) : Error(message) {}
};

struct InvalidConfiguration : Error {
    InvalidConfiguration(const std::string &message) : Error(message) {}
};

struct IOError : Error {
    IOError(const std::string &message) : Error(message) {}
};

struct FormatError : Error {
    FormatError(const std::string &message) : Error(message) {}
};

/** Abandon all operations.
 */
struct AbandonAll : Error {
    AbandonAll(const std::string &message) : Error(message) {}
};

struct EmptyImage : Error {
    EmptyImage(const std::string &message) : Error(message) {}
};

struct FullImage : Error {
    FullImage(const std::string &message) : Error(message) {}
};

struct EmptyDebugMask : Error {
    EmptyDebugMask(const std::string &message) : Error(message) {}
};

struct EmptyGeoData : Error {
    EmptyGeoData(const std::string &message) : Error(message) {}
};

typedef http::NotFound NotFound;
typedef http::ServiceUnavailable Unavailable;
typedef http::InternalServerError InternalError;
typedef http::RequestAborted RequestAborted;
typedef http::BadRequest BadRequest;

#endif // mapproxy_error_hpp_included_
