//
// Created by quantum on 7/5/21.
//

#ifndef CORE_H
#define CORE_H

#include "Instrumentor.h"
#include "Log.h"

template<typename T>
using Ref = std::shared_ptr<T>;

template<typename T>
using Scope = std::unique_ptr<T>;

#endif //CORE_H
