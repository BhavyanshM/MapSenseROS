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

      uint8_t CreateLoadBufferFloat(float* params, uint32_t count);
      uint8_t CreateLoadBufferUnsignedInt(uint32_t *params, uint32_t count);

      uint8_t CreateLoadReadOnlyImage2D_R16(uint16_t *depthBuffer, uint16_t width, uint16_t height);
      uint8_t CreateLoadReadOnlyImage2D_RGBA8(uint8_t *colorBuffer, uint16_t width, uint16_t height);

      uint8_t CreateReadWriteImage2D_R8(uint16_t width, uint16_t height);
      uint8_t CreateReadWriteImage2D_R16(uint16_t width, uint16_t height);
      uint8_t CreateReadWriteImage2D_RFloat(uint16_t width, uint16_t height);
      uint8_t CreateReadWriteImage2D_RGBA8(uint16_t width, uint16_t height);

      uint8_t CreateBufferInt(uint32_t count);

      void ReadImage(uint8_t image, const cl::size_t<3>& region, void* cpuBufferPtr);
      void ReadBuffer(uint8_t buffer, int *cpuBufferPtr, int size);

      void Reset();
      void SetArgument(const std::string& kernel, uint8_t argId, uint8_t bufferId, bool image = false);
      void SetArgumentInt(const std::string& kernel, uint8_t argId, uint32_t value);

   public:
      cl::CommandQueue commandQueue;
      cl::Kernel filterKernel, packKernel, mergeKernel, correspondenceKernel;

   private:
      std::vector<cl::Image2D> images;
      std::vector<cl::Buffer> buffers;

      cl::Context context;
      cl::Event event;
      cl::size_t<3> origin;
};

#endif //OPENCLMANAGER_H
