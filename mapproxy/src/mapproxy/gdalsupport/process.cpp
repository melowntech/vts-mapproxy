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

    LOG(info1) << "Joining process" << id_ << ".";

    int status;
    int options(0);
    if (justTry) { options |= WNOHANG; }
    for (;;) {
        auto res(::waitpid(id_, &status, options));
        if (res < 0) {
            if (EINTR == errno) { continue; }

            std::system_error e(errno, std::system_category());
            LOG(warn1) << "waitpid(2) failed: <" << e.code()
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
