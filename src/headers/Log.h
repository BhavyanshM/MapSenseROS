//
// Created by quantum on 7/3/21.
//

#ifndef LOG_H
#define LOG_H

#include "Core/Log.h"
#include <spdlog/sinks/stdout_color_sinks.h>

#define ENABLE_DEBUG 0

#if ENABLE_DEBUG == 1
#define MS_DEBUG(...) CLAY_LOG_INFO(__VA_ARGS__)
#else
#define MS_DEBUG(...)
#endif

#define MS_INFO(...) CLAY_LOG_INFO(__VA_ARGS__)

class Log
{
   public:
      static void Init()
      {
         spdlog::set_pattern("%^[%T] %n: %v%$");
      }
};

#endif //LOG_H
