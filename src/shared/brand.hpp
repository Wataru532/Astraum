// The program's name, where its data lives, and the tags at the head of its files. The game was called
// StellarForge before Astraum 1.0.0: files with the old tags still load, and the old data folder is copied over
// on the first start (see migrateDataFolder).
#pragma once
#include <string>

#define APP_NAME "Astraum"
#define APP_VERSION "1.1.0"

namespace brand {
constexpr const char* NAME = APP_NAME;
constexpr const char* DATA_FOLDER = "Astraum";       // under %APPDATA%
constexpr const char* LEGACY_FOLDER = "Stellar" "Forge"; // the data folder of the earlier name
constexpr const char* TAG = "ASTRAUM_";              // file tags: ASTRAUM_SAVE, ASTRAUM_CRAFT, ASTRAUM_DESIGN
constexpr const char* LEGACY_TAG = "STELLAR" "FORGE_";

// the first line of a file starts with the tag of this kind ("SAVE", "CRAFT", "DESIGN"), the current or the earlier one
inline bool hasTag(const std::string& line, const char* kind) {
    return line.rfind(std::string(TAG) + kind, 0) == 0 || line.rfind(std::string(LEGACY_TAG) + kind, 0) == 0;
}
// any file of the game (the first line)
inline bool anyTag(const std::string& line) { return line.rfind(TAG, 0) == 0 || line.rfind(LEGACY_TAG, 0) == 0; }
inline std::string tag(const char* kind) { return std::string(TAG) + kind; }
} // namespace brand
