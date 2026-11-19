#include "platform.hpp"
#include "i18n.hpp"

extern "C" __declspec(dllimport) unsigned short __stdcall GetUserDefaultUILanguage(void);

namespace platform {
int systemLanguage() {
    unsigned short primary = GetUserDefaultUILanguage() & 0x3FF;
    return primary == 0x11 ? i18n::JA : (primary == 0x07 ? i18n::DE : i18n::EN);
}
} // namespace platform
