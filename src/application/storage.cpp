#include "storage.hpp"
#include <filesystem>
#include <fstream>
#include "craft.hpp"
#include "diag.hpp"
#include "parts.hpp"
#include "sim.hpp"

bool writeFileAtomic(const std::string& path, const std::string& text) {
    namespace fs = std::filesystem;
    const fs::path p(path), tmp = fs::path(path + ".tmp");
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
    std::ifstream f(std::filesystem::path(path), std::ios::binary | std::ios::ate); // (UTF-8 path)
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

namespace {
bool writePlain(const std::string& path, const std::string& text) {
    std::ofstream f(std::filesystem::path(path), std::ios::binary); // (UTF-8 path: a Japanese name is a Japanese file name)
    if (!f) return false;
    f << text;
    return (bool)f;
}
} // namespace

bool saveDesign(const Design& d, const std::string& path) { return writePlain(path, designToText(d)); }
bool loadDesign(Design& d, const std::string& path) {
    std::string text;
    return readFileLimited(path, text, 1u << 20) && designFromText(d, text);
}
bool saveCraft(const Craft& c, const std::string& path) { return writePlain(path, craftToText(c)); }
bool loadCraft(Craft& c, const std::string& path) {
    std::string text;
    return readFileLimited(path, text, 4u << 20) && craftFromText(c, text);
}
