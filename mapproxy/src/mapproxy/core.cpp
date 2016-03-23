#include "./core.hpp"

Core::Core(Generators &generators)
    : generators_(generators)
{}

void Core::generate_impl(const std::string &location
                         , const Sink::pointer &sink)
{
    (void) location;
    (void) sink;
}
