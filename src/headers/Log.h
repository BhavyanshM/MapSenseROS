//
// Created by quantum on 7/3/21.
//

#ifndef LOG_H
#define LOG_H

#include "spdlog/spdlog.h"
#include <spdlog/sinks/stdout_color_sinks.h>

#define MAPSENSE_LOG_INFO(...) spdlog::info(__VA_ARGS__)
#define MAPSENSE_LOG_ERROR(...) spdlog::error(__VA_ARGS__)
#define MAPSENSE_LOG_TRACE(...) spdlog::trace(__VA_ARGS__)
#define MAPSENSE_LOG_WARN(...) spdlog::warn(__VA_ARGS__)
#define MAPSENSE_LOG_FATAL(...) spdlog::fatal(__VA_ARGS__)

class Log
{
   public:
      static void Init()
      {
         spdlog::set_pattern("%^[%T] %n: %v%$");
      }
};

#endif //LOG_H
