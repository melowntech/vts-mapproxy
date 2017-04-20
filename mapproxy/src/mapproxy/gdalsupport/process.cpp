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

#include <signal.h>
#include <sys/wait.h>

#include <cerrno>
#include <system_error>

#include "dbglog/dbglog.hpp"

#include "utility/process.hpp"

#include "./process.hpp"

Process::ExitCode Process::join(bool justTry)
{

    if (!joinable()) {
        std::system_error e(EINVAL, std::system_category());
        LOG(err3) << "Cannot join non-joinable process.";
        throw e;
    }

    if (id_ == ::getpid()) {
        std::system_error e(EDEADLK, std::system_category());
        LOG(err3) << "Cannot join a process from within.";
        throw e;
    }

    LOG(debug) << (justTry ? "Trying to join process " : "Joining process ")
               << id_ << ".";

    int status;
    int options(0);
    if (justTry) { options |= WNOHANG; }
    for (;;) {
        auto res(::waitpid(id_, &status, options));
        if (res < 0) {
            if (EINTR == errno) { continue; }

            std::system_error e(errno, std::system_category());
            LOG(warn1) << "waitpid(" << id_ << ") failed: <" << e.code()
                       << ", " << e.what() << ">";
            throw e;
        }

        if (!res) {
            // process still running -> Alive
            throw Alive{};
        }
        break;
    }

    LOG(info1) << "Joined process " << id_ << ", status: " << status << ".";

    // reset ID
    id_ = 0;

    if (WIFEXITED(status)) {
        return WEXITSTATUS(status);
    }

    // TODO: handle signals
    return EXIT_FAILURE;
}

void Process::terminate(Id id)
{
    auto res(::kill(id, SIGTERM));

    if (res < 0) {
        std::system_error e(errno, std::system_category());
        LOG(warn1) << "kill(" << id << "SIGTERM) failed: <" << e.code()
                   << ", " << e.what() << ">";
        throw e;
    }
}

void Process::kill(Id id)
{
    auto res(::kill(id, SIGKILL));

    if (res < 0) {
        std::system_error e(errno, std::system_category());
        LOG(warn1) << "kill(" << id << ", SIGKILL) failed: <" << e.code()
                   << ", " << e.what() << ">";
        throw e;
    }
}

void Process::terminate()
{
    if (!joinable()) {
        std::system_error e(EINVAL, std::system_category());
        LOG(err3) << "Cannot join non-joinable process.";
        throw e;
    }

    terminate(id_);
    killed_ = true;
}

void Process::kill()
{
    if (!joinable()) {
        std::system_error e(EINVAL, std::system_category());
        LOG(err3) << "Cannot join non-joinable process.";
        throw e;
    }

    kill(id_);
    killed_ = true;
}

Process::Id Process::run(const std::function<void()> &func
                         , const Flags &flags)
{
    int pflags(utility::SpawnFlag::none);
    if (flags.quickExit()) {
        pflags |= utility::SpawnFlag::quickExit;
    }

    return utility::spawn([=]() -> int { func(); return EXIT_SUCCESS; }
                          , pflags);
}

Process::Id ThisProcess::id()
{
    return ::getpid();
}

Process::Id ThisProcess::parentId()
{
    return ::getppid();
}
