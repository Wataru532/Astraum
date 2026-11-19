// GPU utilisation of this process (Windows performance counters "GPU Engine ... engtype_3D"),
// sampled once per second on a background thread so the frame loop never waits for it.
#pragma once
#include <string>
#include <utility>
#include <vector>

namespace gpumon {
void start();
void stop();
float utilization(); // 0..100 %, or -1 while unknown / unsupported
// ACPI thermal zones (counter set "Thermal Zone Information", no admin rights needed), sampled with
// the GPU load: {zone name, degrees Celsius}. Empty where the system exposes none.
std::vector<std::pair<std::string, float>> temperatures();
} // namespace gpumon
