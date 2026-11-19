// Platform services for the application's ports: the display language of Windows
#pragma once

namespace platform {
int systemLanguage(); // i18n::Lang of the Windows display language (English when it is neither Japanese nor German)
} // namespace platform
