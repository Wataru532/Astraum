// User screenshots (key F2 by default): the frame is read back on the main thread, encoded and written
// as PNG on a worker thread so the game does not stall
#pragma once
#include <filesystem>
#include <string>

namespace screenshot {
// folder the screenshots go to (Pictures\Astraum, or <data>\screenshots as a fallback)
std::filesystem::path folder(const std::string& dataDir);
// read the current back buffer (call after the frame is drawn, before EndDrawing); returns the file
// name that will be written, or "" when the capture was refused (too many pending writes)
std::string capture(const std::string& dataDir);
// wait for pending writes (at shutdown)
void finish();
// last write result: 1 ok, -1 failed, 0 none yet; reset after reading
int takeResult();
// automated tests: write into this folder instead (empty = normal)
void setFolderOverride(const std::filesystem::path& dir);
} // namespace screenshot
