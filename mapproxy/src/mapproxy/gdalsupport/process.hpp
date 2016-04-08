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

    Process() : id_() {}
    Process(Process &&other);

    template<typename Function, typename ...Args>
    Process(const Flags &flags, Function &&f, Args &&...args);

    // template<typename Function, typename ...Args>
    // Process(Function &&f, Args &&...args);

    Process(const Process&) = delete;
    ~Process();

    Process& operator=(Process &&other);
    Process& operator=(Process &other) = delete;

    Id getId() const { return id_; }

    inline bool joinable() const { return id_ > 0; }

    inline void swap(Process &other) {
        std::swap(id_, other.id_);
    }

     /** Joins process. Can throw system_error, see std::thread documentation.
      *
      * \param justTry throws Alive when true and process is still running.
      */
    ExitCode join(bool justTry = false);

private:
    static Id run(const std::function<void()> &func, const Flags &flags);

    Id id_;
};

inline Process::Process(Process &&other)
    : id_(other.id_)
{
    other.id_ = 0;
}

inline Process& Process::operator=(Process &&other)
{
    if (joinable()) { std::terminate(); }
    id_ = other.id_;
    other.id_ = 0;
    return *this;
}

// template<class Function, typename ...Args>
// inline Process::Process(Function &&f, Args &&...args)
// {
//     id_ = run(std::bind<void>(std::forward<Function>(f)
//                               , std::forward<Args>(args)...)
//               , {});
// }

template<class Function, typename ...Args>
inline Process::Process(const Flags &flags, Function &&f, Args &&...args)
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
