// GPU time per part of the frame (OpenGL timestamp queries, read a few frames later so nothing stalls)
#pragma once
#include <string>
#include <vector>

namespace gputime {
void frameBegin();                 // first thing in the frame
void mark(const char* section);    // the GPU time since the previous mark belongs to `section`
void frameEnd();                   // after the last mark
struct Part { std::string name; double gpuMs, cpuMs; };
std::vector<Part> averages();      // over the frames since reset()
void reset();
void shutdown();
bool available();
void setEnabled(bool on); // off: no queries and no batch flushes (the default)
} // namespace gputime
