#ifndef SRC_REGIONFINDER_H
#define SRC_REGIONFINDER_H

#include <CL/cl.hpp>

class RegionFinder {
public:
    cl::Kernel kern;
    cl::Context context;
    cl::CommandQueue queue;
    cl::Image2D imgInBuf, imgOutBuf1, imgOutBuf2;
    cl::ImageFormat uint8_img, float_img;
    cl::size_t<3> origin, size;

    RegionFinder(cl::Kernel kernel);
};


#endif //SRC_REGIONFINDER_H
