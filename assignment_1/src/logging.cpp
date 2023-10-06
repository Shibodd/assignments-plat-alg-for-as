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


}