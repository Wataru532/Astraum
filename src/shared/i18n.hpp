// Localisation: English (key), Japanese, German
#pragma once
#include <string>
#include <vector>

namespace i18n {
enum Lang { EN = 0, JA = 1, DE = 2, COUNT = 3 };
void setLang(int lang);
int lang();
const char* langName(int lang);           // native name of a language
// Translate an English source string. Unknown strings are returned unchanged.
std::string tr(const std::string& en);
// Translate a template and substitute {0}, {1}, {2}
std::string trf(const std::string& en, const std::string& a, const std::string& b = "", const std::string& c = "");
// All codepoints needed to render the given language
std::vector<int> codepoints(int lang);
} // namespace i18n

using i18n::tr;
using i18n::trf;
