// Small Windows shell helpers (kept in their own file: windows.h clashes with raylib names)
#pragma once
#include <filesystem>

namespace winshell {
// the user's Roaming AppData folder (Unicode-safe via SHGetKnownFolderPath); empty if unavailable
std::filesystem::path appDataDir();
// the user's Pictures folder (follows OneDrive / folder redirection); empty if unavailable
std::filesystem::path picturesDir();
// open a folder in Explorer (only existing directories are opened)
void openFolder(const std::filesystem::path& dir);
// an RCDATA resource embedded in the executable (app.rc); the data stays valid for the process lifetime
bool resource(int id, const void*& data, size_t& size);
} // namespace winshell
