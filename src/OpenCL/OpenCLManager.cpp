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
      std::cout << " No platforms found. Check OpenCL installation!\n";
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
      std::cout << ros::package::getPath("map_sense") + "/kernels/fitting_kernel.cpp" << std::endl;
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
      std::cout << " Error building: " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device) << "\n";
      exit(1);
   }
   commandQueue = cl::CommandQueue(context, default_device);
   filterKernel = cl::Kernel(program, "filterKernel");
   packKernel = cl::Kernel(program, "packKernel");
   mergeKernel = cl::Kernel(program, "mergeKernel");

   printf("OpenCL Initialized Successfully\n");
}
