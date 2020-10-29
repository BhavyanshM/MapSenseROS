#include "../include/GPU.h"

void GPU::initQueueCL(){
    vector<cl::Platform> all_platforms;
    cl::Platform::get(&all_platforms);
    if(all_platforms.size()==0){
        cout<<" No platforms found. Check OpenCL installation!\n";
        exit(1);
    }
    cl::Platform default_platform=all_platforms[0];
    cout << "Using platform: "<<default_platform.getInfo<CL_PLATFORM_NAME>()<<"\n";

    vector<cl::Device> all_devices;
    default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
    if(all_devices.size()==0){
        cout<<" No devices found. Check OpenCL installation!\n";
        exit(1);
    }
    device = all_devices[0];
    cout<< "Using device: "<<device.getInfo<CL_DEVICE_NAME>()<<"\n";


    cl::Context contextFromDevice({device});
    context = contextFromDevice;

    cl::CommandQueue q(context,device);
    this->queue = q;
}

void GPU::loadProgramCL(){
    ifstream t("/home/quantum/Workspace/FastStorage/IHMC_PhD/Research/Visual_SLAM/ImageAlignment/fusion/src/Extras/kernel.cpp");
    stringstream buffer;
    buffer << t.rdbuf();
    string kernel_code = buffer.str();

    cl::Program::Sources sources;
    sources.push_back({kernel_code.c_str(),kernel_code.length()});

    cl::Program programFromContext(context,sources);
    program = cl::Program(context,sources);
    if(program.build({device})!=CL_SUCCESS){
        cout<<" Error building: "<<program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device)<<"\n";
        exit(1);
    }

    cl::Kernel computekernel(program, "compute_update");
    this->kernels["compute_update"] =  computekernel;
    cl::Kernel reduceKernel(program, "reduce_jacobian");
    this->kernels["reduce_jacobian"] = reduceKernel;
    cl::Kernel fillNEKernel(program, "calc_normal_eq");
    this->kernels["calc_normal_eq"] = fillNEKernel;
    cl::Kernel calcGradXYKernel(program, "calc_gradient_xy");
    this->kernels["calc_gradient_xy"] = calcGradXYKernel;
}


