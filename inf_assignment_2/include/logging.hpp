#ifndef LOG_HPP
#define LOG_HPP

#include <iostream>
#include <cstdarg>

namespace logging {

enum class LogLevel {
  Disabled = 0,
  Error = 10,
  Warn = 20,
  Info = 30,
  Debug = 100
};

extern LogLevel logLevel;
void setLogLevel(LogLevel level);

class Logger {
  std::string name;

  void logAtLevel(LogLevel level, const char* levelHumanReadable, const char* fmt, ...) const;

public:
  inline Logger(std::string name) : name(name) { }

  #define MAKE_LEVEL_METHOD(level_enum, method_name, level_printed) \
    template<typename... Args> \
    inline void method_name(const char* fmt, Args... args) const { \
      logAtLevel(level_enum, level_printed, fmt, args...); \
    }

  MAKE_LEVEL_METHOD(LogLevel::Info, info, "INFO")
  MAKE_LEVEL_METHOD(LogLevel::Debug, debug, "DEBUG")
  MAKE_LEVEL_METHOD(LogLevel::Error, error, "ERROR")
  MAKE_LEVEL_METHOD(LogLevel::Warn, warn, "WARN")

  #undef MAKE_LEVEL_METHOD
}; // Logger

} // namespace logging

#endif // !LOG_HPP