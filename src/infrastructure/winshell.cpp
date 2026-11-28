#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <objbase.h>
#include <shlobj.h>
#include <shellapi.h>
#include "winshell.hpp"

namespace winshell {
namespace {
struct ComScope {
    bool uninit = false;
    ComScope() {
        HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
        uninit = SUCCEEDED(hr);
    }
    ~ComScope() {
        if (uninit) CoUninitialize();
    }
};

std::filesystem::path knownFolder(REFKNOWNFOLDERID id) {
    ComScope com;
    PWSTR p = nullptr;
    std::filesystem::path out;
    if (SUCCEEDED(SHGetKnownFolderPath(id, 0, nullptr, &p)) && p) out = std::filesystem::path(p);
    if (p) CoTaskMemFree(p);
    return out;
}
} // namespace

std::filesystem::path appDataDir() { return knownFolder(FOLDERID_RoamingAppData); }

std::filesystem::path picturesDir() { return knownFolder(FOLDERID_Pictures); }

void openFolder(const std::filesystem::path& dir) {
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec)) return;
    ComScope com;
    // the path is passed as the file to open (a directory -> Explorer); no command line is built
    ShellExecuteW(nullptr, L"explore", dir.wstring().c_str(), nullptr, nullptr, SW_SHOWNORMAL);
}

bool resource(int id, const void*& data, size_t& size) {
    HRSRC r = FindResourceW(nullptr, MAKEINTRESOURCEW(id), MAKEINTRESOURCEW(10)); // RT_RCDATA
    if (!r) return false;
    HGLOBAL g = LoadResource(nullptr, r);
    if (!g) return false;
    data = LockResource(g);
    size = SizeofResource(nullptr, r);
    return data && size > 0;
}

} // namespace winshell
