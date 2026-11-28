// The library of saved designs and crafts: file names from the players' names, the listing of a folder
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include "app.hpp"
#include "brand.hpp"
#include "i18n.hpp"
#include "storage.hpp"

namespace {
// the UTF-8 character at s[i] (valid text: cleanLineText first): its byte count, its code point in cp
size_t u8char(const std::string& s, size_t i, uint32_t& cp) {
    unsigned char c = (unsigned char)s[i];
    size_t n = c < 0x80 ? 1 : c < 0xE0 ? 2 : c < 0xF0 ? 3 : 4;
    if (i + n > s.size()) { cp = 0xFFFD; return s.size() - i; }
    cp = n == 1 ? c : n == 2 ? (c & 0x1Fu) : n == 3 ? (c & 0x0Fu) : (c & 0x07u);
    for (size_t k = 1; k < n; k++) cp = (cp << 6) | ((unsigned char)s[i + k] & 0x3Fu);
    return n;
}
std::string nameHash(const std::string& s) {
    uint32_t h = 2166136261u;
    for (unsigned char c : s) h = (h ^ c) * 16777619u;
    char b[16];
    std::snprintf(b, sizeof b, "_%06x", (unsigned)(h & 0xFFFFFF));
    return b;
}
// device names Windows keeps for itself (with any extension)
bool reservedName(const std::string& o) {
    std::string u;
    for (char c : o) u += (c >= 'a' && c <= 'z') ? char(c - 32) : c;
    if (u == "CON" || u == "PRN" || u == "AUX" || u == "NUL") return true;
    if (u.size() >= 4 && (u.compare(0, 3, "COM") == 0 || u.compare(0, 3, "LPT") == 0)) {
        std::string r = u.substr(3);
        return (r.size() == 1 && r[0] >= '0' && r[0] <= '9') || r == "\xC2\xB9" || r == "\xC2\xB2" || r == "\xC2\xB3";
    }
    return false;
}
} // namespace

// The name itself (Japanese and other scripts kept). Characters Windows does not allow in a file name, the
// invisible ones (zero-width, text-direction overrides) and a device name make it lossy: a hash of the full
// name is added so different designs never share a file.
std::string sanitizeFileName(const std::string& s) {
    std::string t = cleanLineText(s, 48); // (valid UTF-8 on one line, cut at a character boundary)
    std::string o;
    bool lossy = t != s;
    for (size_t i = 0; i < t.size();) {
        uint32_t cp = 0;
        size_t n = u8char(t, i, cp);
        bool ok;
        if (cp < 0x80) {
            char c = (char)cp;
            ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                 (c != 0 && std::strchr("-_ ()[],!&'+=#@~", c) != nullptr);
        } else {
            ok = !(cp <= 0x9F || (cp >= 0x200B && cp <= 0x200F) || (cp >= 0x2028 && cp <= 0x202E) || (cp >= 0x2060 && cp <= 0x206F) ||
                   cp == 0xFEFF || (cp >= 0xFFF9 && cp <= 0xFFFF) || (cp >= 0xD800 && cp <= 0xDFFF) || (cp >= 0xE0000 && cp <= 0xE007F) ||
                   cp > 0x10FFFF);
        }
        if (ok) o.append(t, i, n);
        else lossy = true;
        i += n;
    }
    // no space at either end (Windows drops a trailing one)
    size_t a = o.find_first_not_of(' '), e = o.find_last_not_of(' ');
    std::string tr0 = a == std::string::npos ? std::string() : o.substr(a, e - a + 1);
    if (tr0 != o) lossy = true;
    o = tr0;
    if (o.empty()) { o = "design"; lossy = true; }
    if (reservedName(o)) lossy = true;
    if (lossy) o += nameHash(s);
    return o;
}

// the file name of versions up to 1.1 (still recognised: such a file is renamed to its name when listed)
std::string legacyFileName(const std::string& s) {
    std::string o;
    bool lossy = false; // a character was dropped or the name was cut (spaces -> '_' is not lossy)
    for (size_t i = 0; i < s.size(); i++) {
        char c = s[i];
        if (o.size() >= 40) { lossy = true; break; }
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_') o += c;
        else if (c == ' ') o += '_';
        else lossy = true;
    }
    if (o.empty()) { o = "design"; lossy = true; }
    if (lossy) {
        // characters were dropped (e.g. a Japanese name): add a hash of the full name so different
        // designs never share a file
        uint32_t h = 2166136261u;
        for (unsigned char c : s) h = (h ^ c) * 16777619u;
        char b[16];
        std::snprintf(b, sizeof b, "_%06x", (unsigned)(h & 0xFFFFFF));
        o += b;
    }
    return o;
}

std::string savedDesignName(const std::string& path) {
    std::filesystem::path fp = pathFromUtf8(path);
    if (fp.empty()) return {};
    std::ifstream f(fp, std::ios::binary); // (UTF-8 path)
    if (!f) return {};
    char buf[512]; // (the header and the name line: never more than this)
    f.read(buf, sizeof buf);
    std::string head(buf, (size_t)f.gcount());
    size_t at = 0;
    for (int n = 0; n < 4 && at < head.size(); n++) {
        size_t eol = head.find('\n', at);
        if (eol == std::string::npos) break; // (a cut line: not trusted)
        std::string line = head.substr(at, eol - at);
        at = eol + 1;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (n == 0) {
            if (!brand::anyTag(line)) return {};
            continue;
        }
        if (line.rfind("name ", 0) == 0) return cleanLineText(line.substr(5), 48);
    }
    return {};
}

std::vector<SavedFile> listSavedDesigns(const std::string& dir, const std::string& ext, size_t maxN) {
    namespace fs = std::filesystem;
    std::vector<fs::path> found;
    std::error_code ec;
    fs::path base = pathFromUtf8(dir);
    if (base.empty()) return {};
    for (fs::directory_iterator it(base, ec), end; !ec && it != end; it.increment(ec)) {
        std::error_code ec2;
        if (it->is_regular_file(ec2) && pathToUtf8(it->path().extension()) == ext) found.push_back(it->path());
        if (found.size() >= maxN) break;
    }
    // (renamed after the directory is read: an entry renamed while iterating could come up twice)
    std::vector<SavedFile> out;
    for (const fs::path& p : found) {
        SavedFile s;
        try {
            s.path = pathToUtf8(p);
            if (s.path.empty()) continue;
            s.name = savedDesignName(s.path);
            if (!s.name.empty()) {
                std::string stem = pathToUtf8(p.stem()), want = sanitizeFileName(s.name);
                if (stem != want && stem == legacyFileName(s.name)) {
                    std::error_code ec2;
                    fs::path to = base / pathFromUtf8(want + ext);
                    if (!to.empty() && !fs::exists(to, ec2) && !ec2) {
                        fs::rename(p, to, ec2);
                        if (!ec2) s.path = pathToUtf8(to);
                    }
                }
            }
        } catch (const std::exception&) {
            continue; // (a file name that is not valid Unicode: skipped)
        }
        out.push_back(s);
    }
    auto key = [](const SavedFile& s) {
        std::string k = s.name.empty() ? pathToUtf8(pathFromUtf8(s.path).stem()) : s.name;
        for (char& c : k) if (c >= 'A' && c <= 'Z') c = char(c + 32);
        return k;
    };
    std::sort(out.begin(), out.end(), [&](const SavedFile& a, const SavedFile& b) {
        std::string ka = key(a), kb = key(b);
        return ka != kb ? ka < kb : a.path < b.path;
    });
    return out;
}

