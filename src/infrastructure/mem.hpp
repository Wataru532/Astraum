// Memory budget: the process may use up to ~4 GB. Caches are trimmed before that (or when the
// system itself runs short of RAM) so the game never starts paging.
#pragma once
#include <cstddef>

namespace mem {
constexpr size_t MB = 1024 * 1024;
constexpr size_t BUDGET = 4096 * MB;     // total the user allowed
constexpr size_t SOFT = 3072 * MB;       // start trimming caches here
constexpr size_t CACHE_BUDGET = 768 * MB; // aero-table cache (rebuild cost ~1 s per large craft; = AERO_CACHE_BUDGET of the domain)

enum Pressure { Normal = 0, Soft = 1, Hard = 2 };
struct Status {
    size_t process = 0;      // private bytes of this process
    size_t sysAvail = 0, sysTotal = 0;
    Pressure pressure = Normal;
    int trims = 0;           // number of cache trims so far
    double lastTrim = -1e9;
};

// sample memory (cheap; call once per frame). Returns the pressure level that requires trimming
// now (Normal = nothing to do); rate limited so a trim happens at most every few seconds.
Pressure update(double now);
const Status& status();
// return freed heap pages to the OS after large frees
void compactHeap();
} // namespace mem
