#include "PlanarRegionCalculator.h"


/*
 * The following are the intrinsic parameters of the depth camera as recorded from L515 RealSense
    height: 480
    width: 640
    distortion_model: "plumb_bob"
    D: [0.0, 0.0, 0.0, 0.0, 0.0]
    K: [459.97265625, 0.0, 341.83984375, 0.0, 459.8046875, 249.173828125, 0.0, 0.0, 1.0]
    R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    P: [459.97265625, 0.0, 341.83984375, 0.0, 0.0, 459.8046875, 249.173828125, 0.0, 0.0, 0.0, 1.0, 0.0]
 * */
void PlanarRegionCalculator::fit() {
    ROS_INFO("Color:[%d,%d] Depth:[%d,%d] Output:[%d,%d]", inputColor.cols, inputColor.rows, inputDepth.cols, inputDepth.rows,
             output.getRegionOutput().cols, output.getRegionOutput().rows);

    /* Input Data OpenCL Buffers */
    uint16_t *depthBuffer = reinterpret_cast<uint16_t *>(inputDepth.data);
    uint8_t *colorBuffer = reinterpret_cast<uint8_t *>(inputColor.data);
    cl::Image2D clDepth(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, cl::ImageFormat(CL_R, CL_UNSIGNED_INT16), WIDTH, HEIGHT, 0, depthBuffer);
    cl::Image2D clColor(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, cl::ImageFormat(CL_RGBA, CL_UNSIGNED_INT8), WIDTH, HEIGHT, 0, colorBuffer);

    /* Output Data OpenCL Buffers */
    // cl::Image2D clDebug(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_UNSIGNED_INT16), WIDTH, HEIGHT);
    cl::Image2D clFilterDepth(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_UNSIGNED_INT16), WIDTH, HEIGHT);
    cl::Image2D clOutput_0(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), SUB_W, SUB_H);
    cl::Image2D clOutput_1(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), SUB_W, SUB_H);
    cl::Image2D clOutput_2(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), SUB_W, SUB_H);
    cl::Image2D clOutput_3(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), SUB_W, SUB_H);
    cl::Image2D clOutput_4(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), SUB_W, SUB_H);
    cl::Image2D clOutput_5(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), SUB_W, SUB_H);
    cl::Image2D clOutput_6(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_UNSIGNED_INT8), SUB_W, SUB_H);

    /* Setting Kernel arguments for patch-packing kernel */
    filterKernel.setArg(0, clDepth);
    filterKernel.setArg(1, clColor);
    filterKernel.setArg(2, clFilterDepth);

    packKernel.setArg(0, clFilterDepth);
    packKernel.setArg(1, clOutput_0);
    packKernel.setArg(2, clOutput_1);
    packKernel.setArg(3, clOutput_2);
    packKernel.setArg(4, clOutput_3);
    packKernel.setArg(5, clOutput_4);
    packKernel.setArg(6, clOutput_5);
    packKernel.setArg(7, clOutput_6);

    mergeKernel.setArg(0, clOutput_0);
    mergeKernel.setArg(1, clOutput_1);
    mergeKernel.setArg(2, clOutput_2);
    mergeKernel.setArg(3, clOutput_3);
    mergeKernel.setArg(4, clOutput_4);
    mergeKernel.setArg(5, clOutput_5);
    mergeKernel.setArg(6, clOutput_6);

    // kernel.setArg(4, clDebug); /* For whenever clDebug may be required. */

    /* Setup size for reading patch-wise kernel maps from GPU */
    cl::size_t<3> regionOutputSize;
    regionOutputSize[0] = SUB_W;
    regionOutputSize[1] = SUB_H;
    regionOutputSize[2] = 1;

    /* Output Data Buffers on CPU */
    Mat debug(HEIGHT, WIDTH, CV_16UC1);
    Mat output_0(SUB_H, SUB_W, CV_32FC1);
    Mat output_1(SUB_H, SUB_W, CV_32FC1);
    Mat output_2(SUB_H, SUB_W, CV_32FC1);
    Mat output_3(SUB_H, SUB_W, CV_32FC1);
    Mat output_4(SUB_H, SUB_W, CV_32FC1);
    Mat output_5(SUB_H, SUB_W, CV_32FC1);
    Mat output_6(SUB_H, SUB_W, CV_8UC1);

    /* Deploy the patch-packing and patch-merging kernels patch-wise */
    commandQueue.enqueueNDRangeKernel(filterKernel, cl::NullRange, cl::NDRange(SUB_H, SUB_W), cl::NullRange);
    commandQueue.enqueueNDRangeKernel(packKernel, cl::NullRange, cl::NDRange(SUB_H, SUB_W), cl::NullRange);
    commandQueue.enqueueNDRangeKernel(mergeKernel, cl::NullRange, cl::NDRange(SUB_H, SUB_W), cl::NullRange);

    /* Read the output data from OpenCL buffers into CPU buffers */
    // commandQueue.enqueueReadImage(clDebug, CL_TRUE, origin, size, 0, 0, debug.data);
    commandQueue.enqueueReadImage(clFilterDepth, CL_TRUE, origin, size, 0, 0, debug.data);
    commandQueue.enqueueReadImage(clOutput_0, CL_TRUE, origin, regionOutputSize, 0, 0, output_0.data);
    commandQueue.enqueueReadImage(clOutput_1, CL_TRUE, origin, regionOutputSize, 0, 0, output_1.data);
    commandQueue.enqueueReadImage(clOutput_2, CL_TRUE, origin, regionOutputSize, 0, 0, output_2.data);
    commandQueue.enqueueReadImage(clOutput_3, CL_TRUE, origin, regionOutputSize, 0, 0, output_3.data);
    commandQueue.enqueueReadImage(clOutput_4, CL_TRUE, origin, regionOutputSize, 0, 0, output_4.data);
    commandQueue.enqueueReadImage(clOutput_5, CL_TRUE, origin, regionOutputSize, 0, 0, output_5.data);
    commandQueue.enqueueReadImage(clOutput_6, CL_TRUE, origin, regionOutputSize, 0, 0, output_6.data);

    /* Synchronize OpenCL to CPU. Block CPU until the entire OpenCL command queue has completed. */
    commandQueue.finish();


    // debug.convertTo(debug, -1, 10, 100);
    // namedWindow("DebugOutput", WINDOW_NORMAL);
    // resizeWindow("DebugOutput", (int)(debug.cols*1.5), (int)(debug.rows*1.5));
    // imshow("DebugOutput", debug);
    // waitKey(0);

    /* Combine the CPU buffers into single image with multiple channels */
    Mat in[] = {output_0, output_1, output_2, output_3, output_4, output_5};
    int from_to[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5};
    mixChannels(in, 6, &output.getRegionOutput(), 1, from_to, 6);
    output.setPatchData(output_6);

    int i = 0;
    int j = 0;
    printf("{%hu}", output.getPatchData().at<uint8_t>(2, 2));
}



void PlanarRegionCalculator::init_opencl() {
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;

    size[0] = WIDTH;
    size[1] = HEIGHT;
    size[2] = 1;

    vector<cl::Platform> all_platforms;
    cl::Platform::get(&all_platforms);

    if (all_platforms.size() == 0) {
        cout << " No platforms found. Check OpenCL installation!\n";
        exit(1);
    }
    cl::Platform default_platform = all_platforms[0];
    vector<cl::Device> all_devices;
    default_platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
    cl::Device default_device = all_devices[0];
    context = cl::Context({default_device});

    cl::Program::Sources sources;
    // calculates for each element; C = A + B

    FILE *fp;
    char *source_str;
    size_t source_size, program_size;

    fp = fopen((ros::package::getPath("map_sense") + "/kernels/fitting_kernel.cpp").c_str(), "rb");
    if (!fp) {
        printf("Failed to load kernel\n");
        cout << ros::package::getPath("map_sense") + "/kernels/fitting_kernel.cpp" << endl;
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
    kernel_code = "#define SIZE_X " + to_string(GRID_X) + "\n"
                  + "#define SIZE_Y " + to_string(GRID_Y)
                  + kernel_code;
    sources.push_back({kernel_code.c_str(), kernel_code.length()});
    cl::Program program(context, sources);
    if (program.build({default_device}) != CL_SUCCESS) {
        cout << " Error building: " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device) << "\n";
        exit(1);
    }
    commandQueue = cl::CommandQueue(context, default_device);
    filterKernel = cl::Kernel(program, "filterKernel");
    packKernel = cl::Kernel(program, "packKernel");
    mergeKernel = cl::Kernel(program, "mergeKernel");
}

static void onMouse(int event, int x, int y, int flags, void *userdata) {
    MapFrame out = *((MapFrame *) userdata);
    if (event == EVENT_MOUSEMOVE) {
        printf("[%d,%d]:", y / 8, x / 8);
        printf("%hu ", out.getPatchData().at<uint8_t>(y / 8, x / 8) );
        Vec6f patch = out.getRegionOutput().at<Vec6f>(y/8, x/8);
        printf("Center:(%.3lf, %.3lf, %.3lf), Normal:(%.3lf, %.3lf, %.3lf)\n", patch[3],patch[4],patch[5],patch[0],patch[1],patch[2]);
    }
}

void PlanarRegionCalculator::generate_regions(SensorDataReceiver* receiver){
    this->_dataReceiver = receiver;
    // dataReceiver.get_sample_depth(inputDepth, 0, 0.01);
    // _dataReceiver->load_sample_depth("/home/quantum/Workspace/Storage/Other/Temp/Depth_L515.png", inputDepth);
    // _dataReceiver->load_sample_color("/home/quantum/Workspace/Storage/Other/Temp/Color_L515.png", inputColor);

    _dataReceiver->load_next_frame(inputDepth, inputColor);

    auto start = high_resolution_clock::now();

    this->fit(); // Generate planar regions from depth map and color image.
    this->mapFrameProcessor.generateSegmentation(output, planarRegionList);

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start).count();
    ROS_INFO("Plane Fitting Took: %.2f ms\n", duration / (float) 1000);
}

void PlanarRegionCalculator::launch_tester() {

    SensorDataReceiver dataReceiver;
    // dataReceiver.get_sample_depth(inputDepth, 0, 0.01);
    dataReceiver.load_sample_depth("/home/quantum/Workspace/Storage/Other/Temp/Depth_L515.png", inputDepth);
    dataReceiver.load_sample_color("/home/quantum/Workspace/Storage/Other/Temp/Color_L515.png", inputColor);

    // medianBlur(inputDepth, inputDepth, 5);

    auto start = high_resolution_clock::now();

    this->fit(); // Generate planar regions from depth map and color image.


    mapFrameProcessor.generateSegmentation(output, planarRegionList);

    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start).count();
    ROS_INFO("Plane Fitting Took: %.2f ms\n", duration / (float) 1000);

    //
    // for (int i = 0; i < SUB_H; i++) {
    //     line(inputDepth, cv::Point(0, i * 8), cv::Point(WIDTH, i * 8), Scalar(255, 255, 0), 1);
    // }
    // for (int i = 0; i < SUB_W; i++) {
    //     line(inputDepth, cv::Point(i * 8, 0), cv::Point(i * 8, HEIGHT), Scalar(255, 255, 0), 1);
    // }

    inputDepth.convertTo(inputDepth, -1, 10, 100);
    Mat dispDepth;
    inputDepth.convertTo(dispDepth, CV_8U, 1/256.0);
    cvtColor(dispDepth, dispDepth, COLOR_GRAY2BGR);

    // output.drawGraph(dispDepth);

    // namedWindow("RealSense L515 Depth", WINDOW_NORMAL);
    // resizeWindow("RealSense L515 Depth", (int)(inputDepth.cols*1.5), (int)(inputDepth.rows*1.5));
    // setMouseCallback("RealSense L515 Depth", onMouse, (void *) &output);
    // imshow("RealSense L515 Depth", dispDepth);
    // imshow("RealSense L515 Color", inputColor);
    // int code = waitKeyEx(0);
    //
    // if (code == 1048689) exit(1);
}
