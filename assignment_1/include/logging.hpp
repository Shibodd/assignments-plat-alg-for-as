#ifndef LOG_HPP
#define LOG_HPP

#include <iostream>
#include <cstdarg>

namespace logging {

enum class LogLevel {
  Info = 0,
  Debug = 100
};

extern LogLevel logLevel;
void setLogLevel(LogLevel level);

static inline bool should_log_at_level(LogLevel level) {
  return static_cast<unsigned int>(logLevel) >= static_cast<unsigned int>(LogLevel::Info);
}

class Logger {
  std::string name;

  void logAtLevel(LogLevel level, const char* levelHumanReadable, const char* fmt, ...) {
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

public:
  inline Logger(std::string name) : name(name) { }

  template<typename... Args>
  inline void debug(const char* fmt, Args... args) {
    logAtLevel(LogLevel::Debug, "DEBUG", fmt, args...);
  }

  template<typename... Args>
  inline void info(const char* fmt, Args... args) {
    logAtLevel(LogLevel::Info, "INFO", fmt, args...);
  }
}; // Logger

} // namespace logging

#endif // !LOG_HPP