// Files of the game: saved worlds (maps), stacked designs (.sfd) and free-form crafts (.sfc). The domain turns
// them into text and back; this is where the text meets the disk (UTF-8 paths, size limits, atomic writes).
#pragma once
#include <string>

struct World;
struct Design;
struct Craft;

// writes `text` through a temporary file and a rename: an interrupted write never destroys the old file
bool writeFileAtomic(const std::string& path, const std::string& text);
// the whole file, refused when it is larger than maxBytes (a damaged or foreign file is not read into memory)
bool readFileLimited(const std::string& path, std::string& out, size_t maxBytes);

bool saveWorld(const World& w, const std::string& path);
bool loadWorld(World& w, const std::string& path); // false: unreadable / damaged, the world stays as it was

bool saveDesign(const Design& d, const std::string& path);
bool loadDesign(Design& d, const std::string& path);
bool saveCraft(const Craft& c, const std::string& path);
bool loadCraft(Craft& c, const std::string& path);
