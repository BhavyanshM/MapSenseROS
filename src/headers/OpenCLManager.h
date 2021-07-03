//
// Created by quantum on 7/3/21.
//

#ifndef OPENCLMANAGER_H
#define OPENCLMANAGER_H

#include <CL/cl.hpp>

class OpenCLManager
{
   public:
      OpenCLManager();
      ~OpenCLManager() = default;

   private:
      cl::Kernel filterKernel, packKernel, mergeKernel;
      cl::Context context;
      cl::CommandQueue commandQueue;
      cl::Event event;
      cl::size_t<3> origin;
};

#endif //OPENCLMANAGER_H
