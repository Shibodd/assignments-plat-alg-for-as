#include "../include/logging.hpp"

namespace logging {

LogLevel logLevel;
void setLogLevel(LogLevel level) {
  logLevel = level;
}

#define LL(lvl_enum) if (level >= static_cast<int>(lvl_enum)) { logLevel = lvl_enum; return; }
void setLogLevel(int level) {
  LL(LogLevel::Debug);
  logLevel = LogLevel::Info;
}
#undef LL

void Logger::logAtLevel(LogLevel level, const char* levelHumanReadable, const char* fmt, ...)
{
  if (should_log_at_level(level)) {
    // Preamble
    fprintf(stderr, "[%s] %s: ", name.c_str(), levelHumanReadable);

    // Message
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);

    // Postamble
    fprintf(stderr, "\n");
  }
}

}