// Port: diagnostics of the simulation (breadcrumbs for crash reports). The infrastructure plugs its crash
// reporter in; without a sink the lines are dropped.
#pragma once

namespace diag {
using Sink = void (*)(const char* line);
void setSink(Sink s);
// printf-style line for the trail of the last actions (bounded length)
void breadcrumb(const char* fmt, ...);
} // namespace diag
