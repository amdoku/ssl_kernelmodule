#include <chrono>
#include <cstdint>

static inline int64_t getCurrentTime() {
    using namespace std::chrono;
    // should be platform independent
    // OSX uses us per default, we expect ns
     return time_point_cast<nanoseconds>(system_clock::now()).time_since_epoch().count();
}