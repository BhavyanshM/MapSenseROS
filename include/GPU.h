#ifndef CURVEFITTING_GPU_H
#define CURVEFITTING_GPU_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <CL/cl.hpp>

using namespace std;

class GPU {
public:
    cl::Context context;
    cl::Device device;
    cl::Program program;
    cl::CommandQueue queue;
    map<string, cl::Kernel> kernels;
    map<string, cl::Buffer> buffers;
    map<string, cl::Image2D> images;

    void initQueueCL();
    void loadProgramCL();
};


#endif //CURVEFITTING_GPU_H
