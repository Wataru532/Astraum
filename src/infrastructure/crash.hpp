// Crash reporting: unhandled SEH exceptions, std::terminate and abort write a report file;
// the next start shows it to the user.
#pragma once
#include <string>

namespace crash {
// install handlers; reports go to <dir>\crash-YYYYMMDD-HHMMSS.txt with a "pending" marker
void init(const std::string& dir, const char* version);
// short event log kept in a fixed ring buffer (no allocation in the crash path)
void breadcrumb(const char* fmt, ...);
// current application state shown in the report (screen, vessel, ...)
void setState(const char* fmt, ...);
// report from the previous run that the user has not seen yet
bool pendingReport(std::string& text, std::string& path);
void markSeen();
// last lines of the engine log (raylib TraceLog) included in reports
void logLine(const char* line);
// deliberately crash (self-test of the reporter)
[[noreturn]] void selfTest();
// private bytes committed by this process (memory budget monitoring)
size_t processMemoryBytes();
} // namespace crash
