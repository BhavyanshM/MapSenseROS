//
// Created by quantum on 7/3/21.
//

#ifndef OPENCLMANAGER_H
#define OPENCLMANAGER_H

#include "Core.h"
#include <CL/cl.hpp>

class OpenCLManager
{
   public:
      OpenCLManager();
      ~OpenCLManager() = default;

      uint8_t CreateBufferFloat(float* params, uint16_t count);

      uint8_t CreateImage2D_R16(uint16_t *depthBuffer, uint16_t width, uint16_t height);
      uint8_t CreateImage2D_RGBA8(uint8_t *colorBuffer, uint16_t width, uint16_t height);

      uint8_t CreateOutputImage2D_R8(uint16_t width, uint16_t height);
      uint8_t CreateOutputImage2D_R16(uint16_t width, uint16_t height);
      uint8_t CreateOutputImage2D_RFloat(uint16_t width, uint16_t height);

      void ReadImage(uint8_t image, const cl::size_t<3>& region, void* cpuBufferPtr);

      void Reset();
      void SetArgument(const std::string& kernel, uint8_t argId, uint8_t bufferId, bool image = false);

   public:
      cl::CommandQueue commandQueue;
      cl::Kernel filterKernel, packKernel, mergeKernel;

   private:
      std::vector<cl::Image2D> images;
      std::vector<cl::Buffer> buffers;

      cl::Context context;
      cl::Event event;
      cl::size_t<3> origin;
};

#endif //OPENCLMANAGER_H
