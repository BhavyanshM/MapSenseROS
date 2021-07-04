//
// Created by quantum on 7/3/21.
//

#include "OpenCLManager.h"

OpenCLManager::OpenCLManager()
{
   printf("Initializing OpenCL\n");

   std::vector<cl::Platform> all_platforms;
   cl::Platform::get(&all_platforms);

   if (all_platforms.size() == 0)
   {
      ROS_DEBUG(" No platforms found. Check OpenCL installation!");
      exit(1);
   }
   cl::Platform default_platform = all_platforms[0];
   std::vector<cl::Device> all_devices;
   default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
   cl::Device default_device = all_devices[0];
   context = cl::Context({default_device});

   cl::Program::Sources sources;

   FILE *fp;
   char *source_str;
   size_t source_size, program_size;

   fp = fopen((ros::package::getPath("map_sense") + "/kernels/fitting_kernel.cpp").c_str(), "rb");
   if (!fp)
   {
      printf("Failed to load kernel\n");
      MAPSENSE_LOG_INFO(ros::package::getPath("map_sense") + "/kernels/fitting_kernel.cpp");
      return;
   }

   fseek(fp, 0, SEEK_END);
   program_size = ftell(fp);
   rewind(fp);
   source_str = (char *) malloc(program_size + 1);
   source_str[program_size] = '\0';
   fread(source_str, sizeof(char), program_size, fp);
   fclose(fp);

   std::string kernel_code(source_str);
   sources.push_back({kernel_code.c_str(), kernel_code.length()});
   cl::Program program(context, sources);
   if (program.build({default_device}) != CL_SUCCESS)
   {
      MAPSENSE_LOG_INFO(" Error building: {0}", program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device));
      exit(1);
   }
   commandQueue = cl::CommandQueue(context, default_device);
   filterKernel = cl::Kernel(program, "filterKernel");
   packKernel = cl::Kernel(program, "packKernel");
   mergeKernel = cl::Kernel(program, "mergeKernel");

   printf("OpenCL Initialized Successfully\n");

   origin[0] = 0;
   origin[0] = 0;
   origin[0] = 0;
}

uint8_t OpenCLManager::CreateBufferFloat(float *params, uint16_t count)
{
   ROS_DEBUG("Creating Buffer :%d", buffers.size());
   buffers.emplace_back(cl::Buffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(float) * count, params));
   return buffers.size() - 1;
}

uint8_t OpenCLManager::CreateImage2D_R16(uint16_t *depthBuffer, uint16_t width, uint16_t height)
{
   ROS_DEBUG("Creating Image :%d", images.size());
   images.emplace_back(cl::Image2D(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, cl::ImageFormat(CL_R, CL_UNSIGNED_INT16), width, height, 0, depthBuffer));
   return images.size() - 1;
}

uint8_t OpenCLManager::CreateImage2D_RGBA8(uint8_t *colorBuffer, uint16_t width, uint16_t height)
{
   ROS_DEBUG("Creating Image :%d", images.size());
   images.emplace_back(
         cl::Image2D(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, cl::ImageFormat(CL_RGBA, CL_UNSIGNED_INT8), width, height, 0, colorBuffer));
   return images.size() - 1;
}

uint8_t OpenCLManager::CreateOutputImage2D_R8(uint16_t width, uint16_t height)
{
   ROS_DEBUG("Creating Image :%d", images.size());
   images.emplace_back(cl::Image2D(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_UNSIGNED_INT8), width, height));
   return images.size() - 1;
}

uint8_t OpenCLManager::CreateOutputImage2D_R16(uint16_t width, uint16_t height)
{
   ROS_DEBUG("Creating Image :%d", images.size());
   images.emplace_back(cl::Image2D(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_UNSIGNED_INT16), width, height));
   return images.size() - 1;
}

uint8_t OpenCLManager::CreateOutputImage2D_RFloat(uint16_t width, uint16_t height)
{
   ROS_DEBUG("Creating Image :%d", images.size());
   images.emplace_back(cl::Image2D(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), width, height));
   return images.size() - 1;
}

void OpenCLManager::ReadImage(uint8_t image, const cl::size_t<3>& region, void *cpuBufferPtr)
{
   ROS_DEBUG("Reading Image :%d", image);
   commandQueue.enqueueReadImage(images[image], CL_TRUE, origin, region, 0, 0, cpuBufferPtr);
}

void OpenCLManager::Reset()
{
   images.clear();
   buffers.clear();
}

void OpenCLManager::SetArgument(const std::string& kernel, uint8_t argId, uint8_t bufferId, bool image)
{
   if(kernel == "filterKernel")
      if(image)
      {
         filterKernel.setArg(argId, images[bufferId]);
      }
      else
      {
         filterKernel.setArg(argId, buffers[bufferId]);
      }

   if(kernel == "packKernel")
      if(image)
      {
         packKernel.setArg(argId, images[bufferId]);
      }
      else
      {
         packKernel.setArg(argId, buffers[bufferId]);
      }

   if(kernel == "mergeKernel")
      if(image)
      {
         mergeKernel.setArg(argId, images[bufferId]);
      }
      else
      {
         mergeKernel.setArg(argId, buffers[bufferId]);
      }
}


