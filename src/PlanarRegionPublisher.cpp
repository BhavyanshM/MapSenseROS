#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "map_sense/PlanarRegions.h"
#include "geometry_msgs/PoseStamped.h"

#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"

#include <iostream>
#include <CL/cl.hpp>
#include "math.h"
#include <sstream>
#include <random>

using namespace ros;
using namespace std;
using namespace chrono;
using namespace cv;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace map_sense;

#define WIDTH 640
#define HEIGHT 480
#define GRID_X 8
#define GRID_Y 8
#define SUB_W 80
#define SUB_H 60



Publisher RGBDPosePub;
Publisher planarRegionPub;

cl::Kernel packKernel, mergeKernel;
cl::Context context;
cl::CommandQueue commandQueue;
cl::Image2D imgInBuf, imgOutBuf1, imgOutBuf2;
cl::ImageFormat uint8_img, float_img;
cl::size_t<3> origin, size;
cl::Event event;

ImageConstPtr colorMessage;
ImageConstPtr depthMessage;

void get_sample_depth(Mat mat);

void get_sample_color(Mat mat);

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
void fit(const Mat &color, const Mat &depth, Mat &regionOutput) {
    ROS_INFO("Color:[%d,%d] Depth:[%d,%d] Output:[%d,%d]", color.cols, color.rows, depth.cols, depth.rows,
             regionOutput.cols, regionOutput.rows);

    // Mat debug(HEIGHT, WIDTH, CV_16UC1);

    uint16_t *prevDepthBuffer = reinterpret_cast<uint16_t *>(depth.data);
    cl::Image2D clDepth(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, cl::ImageFormat(CL_R, CL_UNSIGNED_INT16),
                        WIDTH, HEIGHT, 0, prevDepthBuffer);
    // cl::Image2D clDebug(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_UNSIGNED_INT16), WIDTH, HEIGHT);

    cl::Image2D clOutput_0(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), SUB_W, SUB_H);
    cl::Image2D clOutput_1(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), SUB_W, SUB_H);
    cl::Image2D clOutput_2(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), SUB_W, SUB_H);
    cl::Image2D clOutput_3(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), SUB_W, SUB_H);
    cl::Image2D clOutput_4(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), SUB_W, SUB_H);
    cl::Image2D clOutput_5(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), SUB_W, SUB_H);

    packKernel.setArg(0, clDepth);
    packKernel.setArg(1, clOutput_0);
    packKernel.setArg(2, clOutput_1);
    packKernel.setArg(3, clOutput_2);
    packKernel.setArg(4, clOutput_3);
    packKernel.setArg(5, clOutput_4);
    packKernel.setArg(6, clOutput_5);
    // kernel.setArg(4, clDebug);

    cl::size_t<3> regionOutputSize;
    regionOutputSize[0] = SUB_W;
    regionOutputSize[1] = SUB_H;
    regionOutputSize[2] = 1;

    Mat output_0(SUB_H, SUB_W, CV_32FC1);
    Mat output_1(SUB_H, SUB_W, CV_32FC1);
    Mat output_2(SUB_H, SUB_W, CV_32FC1);
    Mat output_3(SUB_H, SUB_W, CV_32FC1);
    Mat output_4(SUB_H, SUB_W, CV_32FC1);
    Mat output_5(SUB_H, SUB_W, CV_32FC1);

    commandQueue.enqueueNDRangeKernel(packKernel, cl::NullRange, cl::NDRange(SUB_H, SUB_W), cl::NullRange);
    // commandQueue.enqueueNDRangeKernel(mergeKernel, cl::NullRange, cl::NDRange(SUB_H, SUB_W), cl::NullRange);

    // commandQueue.enqueueReadImage(clDebug, CL_TRUE, origin, size, 0, 0, debug.data);
    commandQueue.enqueueReadImage(clOutput_0, CL_TRUE, origin, regionOutputSize, 0, 0, output_0.data);
    commandQueue.enqueueReadImage(clOutput_1, CL_TRUE, origin, regionOutputSize, 0, 0, output_1.data);
    commandQueue.enqueueReadImage(clOutput_2, CL_TRUE, origin, regionOutputSize, 0, 0, output_2.data);
    commandQueue.enqueueReadImage(clOutput_3, CL_TRUE, origin, regionOutputSize, 0, 0, output_3.data);
    commandQueue.enqueueReadImage(clOutput_4, CL_TRUE, origin, regionOutputSize, 0, 0, output_4.data);
    commandQueue.enqueueReadImage(clOutput_5, CL_TRUE, origin, regionOutputSize, 0, 0, output_5.data);


    commandQueue.finish();

    // debug.convertTo(debug, -1, 4, 100);
    // imshow("Output", debug);

    Mat in[] = {output_0, output_1, output_2, output_3, output_4, output_5};
    int from_to[] = {0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5};
    mixChannels(in, 6, &regionOutput, 1, from_to, 6);

    int i = 0;
    int j = 0;
    cout << regionOutput.at<Vec3f>(2, 2) << endl;
}

void depthCallback(const ImageConstPtr &depthMsg) {
    depthMessage = depthMsg;
}

void colorCallback(const sensor_msgs::ImageConstPtr &colorMsg) {
    colorMessage = colorMsg;
}

void processDataCallback(const TimerEvent &) {
    cv_bridge::CvImagePtr img_ptr_depth;
    cv_bridge::CvImagePtr img_ptr_color;
    if (colorMessage != nullptr && depthMessage != nullptr) {
        try {
            // ROS_INFO("Callback: Color:%d Depth:%d", colorMessage->header.stamp.sec, depthMessage->header.stamp.sec);

            img_ptr_color = cv_bridge::toCvCopy(*colorMessage, image_encodings::TYPE_8UC3);
            Mat colorImg = img_ptr_color->image;

            img_ptr_depth = cv_bridge::toCvCopy(*depthMessage, image_encodings::TYPE_16UC1);
            Mat depthImg = img_ptr_depth->image;
            depthImg.convertTo(depthImg, -1, 10, 100);

            Mat output(depthImg.rows, depthImg.cols, CV_16UC1);
            // fit(colorImg, depthImg, output);

            imshow("RealSense L515 Depth", depthImg);
            imshow("RealSense L515 Color", colorImg);
            int code = waitKeyEx(32);
            if (code == 1048689) exit(1);

        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("Could not convert to image!");
        }
    }
}


void launch_ros_node(int argc, char **argv) {
    init(argc, argv, "PlanarRegionPublisher");
    NodeHandle nh;
    RGBDPosePub = nh.advertise<PoseStamped>("rgbd_pose", 100);
    planarRegionPub = nh.advertise<PlanarRegions>("/map/regions/test", 10);
    Subscriber subDepth = nh.subscribe("/camera/depth/image_rect_raw", 3, depthCallback);
    Subscriber subColor = nh.subscribe("/camera/color/image_raw", 3, colorCallback);
    Timer timer1 = nh.createTimer(Duration(0.1), processDataCallback);
    spin();
}

void init_opencl() {
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
    packKernel = cl::Kernel(program, "segmentKernel");
}

void onMouse(int event, int x, int y, int flags, void *userdata) {
    Mat out = *((Mat *) userdata);
    if (event == EVENT_MOUSEMOVE) {
        printf("[%d,%d]:", y / 8, x / 8);
        cout << out.at<Vec6f>(y / 8, x / 8) << endl;
    }
}

void launch_tester() {
    Mat inputDepth(HEIGHT, WIDTH, CV_16UC1);
    Mat inputColor(HEIGHT, WIDTH, CV_8UC3);
    Mat regionOutput(SUB_H, SUB_W, CV_32FC(6));


    get_sample_depth(inputDepth);
    get_sample_color(inputColor);

    auto start = high_resolution_clock::now();
    fit(inputColor, inputDepth, regionOutput);
    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start).count();
    ROS_INFO("Plane Fitting Took: %.2f ms\n", duration / (float) 1000);

    // imshow("RealSense L515 Color", inputColor);

    inputDepth.convertTo(inputDepth, -1, 4, 100);

    for (int i = 0; i < SUB_H; i++) {
        line(inputDepth, cv::Point(0, i * 8), cv::Point(WIDTH, i * 8), Scalar(255, 255, 0), 1);
    }
    for (int i = 0; i < SUB_W; i++) {
        line(inputDepth, cv::Point(i * 8, 0), cv::Point(i * 8, HEIGHT), Scalar(255, 255, 0), 1);
    }

    namedWindow("RealSense L515 Depth", 1);
    setMouseCallback("RealSense L515 Depth", onMouse, (void *) &regionOutput);

    imshow("RealSense L515 Depth", inputDepth);


    int code = waitKeyEx(0);
    // if (code == 1048689) exit(1);
}

void get_sample_depth(Mat depth) {
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 0.01);
    for (int i = 0; i < depth.cols; i++) {
        for (int j = 0; j < depth.rows; j++) {
            float d = 10.04;
            d += distribution(generator);
            if (160 < i && i < 350 && 160 < j && j < 350) {
                d = 0.008 * i + 0.014 * j + 0.12;
                depth.at<short>(j, i) = d * 1000;
            } else {
                depth.at<short>(j, i) = d * 1000;
            }
        }
    }
}

void get_sample_color(Mat color) {
    for (int i = 0; i < color.rows; i++) {
        for (int j = 0; j < color.cols; j++) {
            color.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
        }
    }
}

int main (int argc, char** argv){

    // export ROSCONSOLE_FORMAT='${time:format string}'
    // export ROSCONSOLE_FORMAT='[${severity}] [${time}] [${node}:${line}]: ${message}'

    init_opencl();
    launch_ros_node(argc, argv);
    // launch_tester();
    return 0;
}
