#include "../include/logging.hpp"

namespace logging {

LogLevel logLevel;
void setLogLevel(LogLevel level) {
  static Logger logger("setLogLevel");
  if (logLevel != level) {
    logLevel = level;
    logger.info("Log level changed to %d.", static_cast<unsigned int>(logLevel));
  }
}

#define LL(lvl_enum) if (level >= static_cast<unsigned int>(lvl_enum)) { setLogLevel(lvl_enum); return; }
void setLogLevel(int level) {
  LL(LogLevel::Debug);
  LL(LogLevel::Info);
  LL(LogLevel::Warn);
  LL(LogLevel::Error);
  logLevel = LogLevel::Info;
}
#undef LL

static inline bool should_log_at_level(LogLevel level) {
  return static_cast<unsigned int>(logLevel) >= static_cast<unsigned int>(level);
}

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