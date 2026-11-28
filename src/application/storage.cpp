#include "storage.hpp"
#include <filesystem>
#include <fstream>
#include "craft.hpp"
#include "diag.hpp"
#include "parts.hpp"
#include "sim.hpp"

std::filesystem::path pathFromUtf8(const std::string& utf8) {
    try {
        std::u8string u8(utf8.size(), u8'\0');
        for (size_t i = 0; i < utf8.size(); i++) u8[i] = (char8_t)(unsigned char)utf8[i];
        return std::filesystem::path(u8);
    } catch (...) {
        return std::filesystem::path();
    }
}

std::string pathToUtf8(const std::filesystem::path& p) {
    try {
        std::u8string u8 = p.u8string();
        std::string out(u8.size(), '\0');
        for (size_t i = 0; i < u8.size(); i++) out[i] = (char)u8[i];
        return out;
    } catch (...) {
        return {};
    }
}

bool writeFileAtomic(const std::string& path, const std::string& text) {
    namespace fs = std::filesystem;
    const fs::path p = pathFromUtf8(path), tmp = pathFromUtf8(path + ".tmp");
    if (p.empty() || tmp.empty()) return false;
    std::error_code ec;
    {
        std::ofstream o(tmp, std::ios::binary);
        if (!o) return false;
        o.write(text.data(), (std::streamsize)text.size());
        o.flush();
        if (!o) {
            o.close();
            fs::remove(tmp, ec);
            return false;
        }
    }
    fs::rename(tmp, p, ec); // replaces the previous file atomically
    if (ec) {
        fs::remove(tmp, ec);
        return false;
    }
    return true;
}

bool readFileLimited(const std::string& path, std::string& out, size_t maxBytes) {
    std::filesystem::path p = pathFromUtf8(path);
    if (p.empty()) return false;
    std::ifstream f(p, std::ios::binary | std::ios::ate); // (UTF-8 path)
    if (!f) return false;
    std::streamoff size = f.tellg(); // the size before reading anything
    if (size < 0 || (unsigned long long)size > maxBytes) return false;
    f.seekg(0);
    std::string text((size_t)size, '\0');
    if (size > 0 && !f.read(&text[0], size)) return false;
    out = std::move(text);
    return true;
}

bool saveWorld(const World& w, const std::string& path) {
    diag::breadcrumb("save game");
    return writeFileAtomic(path, w.serialize());
}

bool loadWorld(World& w, const std::string& path) {
    std::string text;
    if (!readFileLimited(path, text, 256u << 20)) return false;
    return w.deserialize(text);
}

bool saveDesign(const Design& d, const std::string& path) { return writeFileAtomic(path, designToText(d)); }
bool loadDesign(Design& d, const std::string& path) {
    std::string text;
    return readFileLimited(path, text, 1u << 20) && designFromText(d, text);
}
bool saveCraft(const Craft& c, const std::string& path) { return writeFileAtomic(path, craftToText(c)); }
bool loadCraft(Craft& c, const std::string& path) {
    std::string text;
    return readFileLimited(path, text, 4u << 20) && craftFromText(c, text);
}
