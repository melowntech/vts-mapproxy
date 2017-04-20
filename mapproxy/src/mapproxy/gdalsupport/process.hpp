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

#ifndef mapproxy_process_hpp_included_
#define mapproxy_process_hpp_included_

#include <exception>
#include <functional>

class Process {
public:
    typedef int Id;
    typedef int ExitCode;

    class Flags {
    public:
        Flags() : quickExit_(false) {}
        Flags& quickExit(bool value) { quickExit_ = value; return *this; }
        bool quickExit() const { return quickExit_; }

    private:
        bool quickExit_;
    };

    struct Alive {};

    Process() : id_(), killed_(false) {}
    Process(Process &&other);

    template<typename Function, typename ...Args>
    Process(const Flags &flags, Function &&f, Args &&...args);

    // template<typename Function, typename ...Args>
    // Process(Function &&f, Args &&...args);

    Process(const Process&) = delete;
    ~Process();

    Process& operator=(Process &&other);
    Process& operator=(Process &other) = delete;

    Id id() const { return id_; }

    inline bool joinable() const { return id_ > 0; }

    inline void swap(Process &other) {
        std::swap(id_, other.id_);
    }

    /** Joins process. Can throw system_error, see std::thread documentation.
     *
     * \param justTry throws Alive when true and process is still running.
     */
    ExitCode join(bool justTry = false);

    /** Terminate process (soft kill).
     */
    void terminate();

    /** Kill the process (hard kill).
     */
    void kill();

    bool killed() const { return killed_; }

    static void terminate(Id id);

    static void kill(Id id);

private:
    static Id run(const std::function<void()> &func, const Flags &flags);

    Id id_;

    bool killed_;
};

struct ThisProcess {
    static Process::Id id();
    static Process::Id parentId();
};

// inlines

inline Process::Process(Process &&other)
    : id_(other.id_), killed_(other.killed_)
{
    other.id_ = 0;
    other.killed_ = false;
}

inline Process& Process::operator=(Process &&other)
{
    if (joinable()) { std::terminate(); }
    id_ = other.id_;
    killed_ = other.killed_;
    other.id_ = 0;
    other.killed_ = false;
    return *this;
}

template<class Function, typename ...Args>
inline Process::Process(const Flags &flags, Function &&f, Args &&...args)
    : killed_(false)
{
    id_ = run(std::bind<void>(std::forward<Function>(f)
                              , std::forward<Args>(args)...)
              , flags);
}

inline Process::~Process()
{
    if (joinable()) { std::terminate(); }
}

// inlines

#endif // mapproxy_process_hpp_included_
