#include <map_sense/RawGPUPlanarRegionList.h>
#include <AppUtils.h>
#include "PlanarRegionCalculator.h"
#include "imgui.h"

PlanarRegionCalculator::PlanarRegionCalculator(int argc, char **argv, ApplicationState& app) : app(app)
{
   std::cout << "PlanarRegionCalculator Created" << std::endl;
   ROS_INFO("Creating PlanarRegionCalculator");
   _mapFrameProcessor = new MapFrameProcessor(app);
   _mapFrameProcessor->init(app);
   inputDepth = cv::Mat(app.INPUT_HEIGHT, app.INPUT_WIDTH, CV_16UC1);
   inputColor = cv::Mat(app.INPUT_HEIGHT, app.INPUT_WIDTH, CV_8UC3);
   origin[0] = 0;
   origin[0] = 0;
   origin[0] = 0;
}

void PlanarRegionCalculator::ImGuiUpdate(ApplicationState& appState)
{
   if (ImGui::BeginTabItem("Planar Regions"))
   {
      /* Display 2D */
      ImGui::Text("Input:%d,%d Patch:%d,%d Level:%d", appState.INPUT_HEIGHT, appState.INPUT_WIDTH, appState.PATCH_HEIGHT, appState.PATCH_WIDTH,
                  appState.KERNEL_SLIDER_LEVEL);
      ImGui::Checkbox("Generate Regions", &appState.GENERATE_REGIONS);
      ImGui::Checkbox("Filter", &appState.FILTER_SELECTED);
      ImGui::Checkbox("Early Gaussian", &appState.EARLY_GAUSSIAN_BLUR);
      ImGui::SliderInt("Gaussian Size", &appState.GAUSSIAN_SIZE, 1, 8);
      ImGui::SliderInt("Gaussian Sigma", &appState.GAUSSIAN_SIGMA, 1, 20);
      ImGui::SliderFloat("Distance Threshold", &appState.MERGE_DISTANCE_THRESHOLD, 0.0f, 0.1f);
      ImGui::SliderFloat("Angular Threshold", &appState.MERGE_ANGULAR_THRESHOLD, 0.0f, 1.0f);
      ImGui::Checkbox("Components", &appState.SHOW_REGION_COMPONENTS);
      ImGui::Checkbox("Boundary", &appState.SHOW_BOUNDARIES);
      ImGui::Checkbox("Internal", &appState.SHOW_PATCHES);
      if (ImGui::Button("Hide Display"))
      {
         cv::destroyAllWindows();
      }
      ImGui::SliderFloat("Display Window Size", &appState.DISPLAY_WINDOW_SIZE, 0.1, 5.0);
      ImGui::SliderFloat("Depth Brightness", &appState.DEPTH_BRIGHTNESS, 1.0, 100.0);
      ImGui::Checkbox("Visual Debug", &appState.VISUAL_DEBUG);
      ImGui::SliderInt("Visual Debug Delay", &appState.VISUAL_DEBUG_DELAY, 1, 100);
      ImGui::Checkbox("Show Edges", &appState.SHOW_REGION_EDGES);
      ImGui::SliderInt("Skip Edges", &appState.NUM_SKIP_EDGES, 1, 20);
      ImGui::SliderFloat("Magnum Patch Scale", &appState.MAGNUM_PATCH_SCALE, 0.001f, 0.04f);

      ImGui::Checkbox("Render 3D", &_render);

      ImGui::EndTabItem();
   }
}

void PlanarRegionCalculator::Render()
{
   if (this->app.SHOW_REGION_COMPONENTS)
   {
      ROS_INFO("Appending Region Components");
      AppUtils::DisplayImage(_mapFrameProcessor->debug, app);
      //      appUtils.appendToDebugOutput(this->_mapFrameProcessor.debug);
   }
}

uint8_t PlanarRegionCalculator::CreateParameterBuffer(const ApplicationState& app)
{
   float params[] = {(float) app.FILTER_DISPARITY_THRESHOLD, app.MERGE_ANGULAR_THRESHOLD, app.MERGE_DISTANCE_THRESHOLD, (float) app.PATCH_HEIGHT,
                     (float) app.PATCH_WIDTH, (float) app.SUB_H, (float) app.SUB_W, app.DEPTH_FX, app.DEPTH_FY, app.DEPTH_CX, app.DEPTH_CY,
                     (float) app.FILTER_KERNEL_SIZE, (float) app.FILTER_SUB_H, (float) app.FILTER_SUB_W, (float) app.INPUT_HEIGHT, (float) app.INPUT_WIDTH};

   ROS_INFO("GenerateRegions:(%d, %d, %d, %d, %d, %d) Filter:(%d,%d):%d", app.INPUT_HEIGHT, app.INPUT_WIDTH, app.PATCH_HEIGHT, app.PATCH_WIDTH, app.SUB_H,
            app.SUB_W, app.FILTER_SUB_H, app.FILTER_SUB_W, app.FILTER_KERNEL_SIZE);

   return _openCL->CreateLoadBufferFloat(params, sizeof(params) / sizeof(float));
}

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
bool PlanarRegionCalculator::GeneratePatchGraphFromDepth(ApplicationState& appState)
{
   MAPSENSE_PROFILE_FUNCTION();
   ROS_INFO("Generating Patch Graph on GPU: Color:[%d,%d] Depth:[%d,%d] Output:[%d,%d]", inputColor.cols, inputColor.rows, inputDepth.cols, inputDepth.rows,
            output.getRegionOutput().cols, output.getRegionOutput().rows);

   if (inputDepth.rows <= 0 || inputDepth.cols <= 0 || inputDepth.dims <= 0)
   {
      ROS_INFO("Depth image width, height, or dimensions are zero! Skipping Frame.");
      return false;
   }

   this->app = appState;
   uint8_t paramsBuffer = CreateParameterBuffer(appState);

   cv::Mat depthMat, colorMat;
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCV::CloneAndBlur");
      depthMat = inputDepth.clone();
      colorMat = inputColor.clone();

      if (appState.EARLY_GAUSSIAN_BLUR)
         GaussianBlur(depthMat, depthMat, cv::Size(appState.GAUSSIAN_SIZE * 2 + 1, appState.GAUSSIAN_SIZE * 2 + 1), appState.GAUSSIAN_SIGMA);
   }

   /* Input Data OpenCL Buffers */
   uint16_t *depthBuffer = reinterpret_cast<uint16_t *>(depthMat.data);
   uint8_t clDepth = _openCL->CreateLoadReadOnlyImage2D_R16(depthBuffer, appState.INPUT_WIDTH, appState.INPUT_HEIGHT);
   //   uint8_t *colorBuffer = reinterpret_cast<uint8_t *>(colorMat.data);
   //   uint8_t clColor = _openCL->CreateLoadReadOnlyImage2D_RGBA8(colorBuffer, appState.INPUT_WIDTH, appState.INPUT_HEIGHT);

   /* Output Data OpenCL Buffers */
   uint8_t clFilterDepth = _openCL->CreateReadWriteImage2D_R16(appState.INPUT_WIDTH, appState.INPUT_HEIGHT);
   uint8_t clBuffer_nx = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_ny = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_nz = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_gx = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_gy = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_gz = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_graph = _openCL->CreateReadWriteImage2D_R8(appState.SUB_W, appState.SUB_H);

   ROS_INFO("Created All Input Images.");

   /* Setting Kernel arguments for patch-packing kernel */
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::SetArgument(s)");

      uint8_t clDepthBuffer = appState.FILTER_SELECTED ? clFilterDepth : clDepth;

      std::vector<uint8_t> argsImgFilter = {clDepth, clFilterDepth, clBuffer_nx};
      for (uint8_t i = 0; i < argsImgFilter.size(); i++)
         _openCL->SetArgument("filterKernel", i, argsImgFilter[i], true);
      _openCL->SetArgument("filterKernel", argsImgFilter.size(), paramsBuffer);

      std::vector<uint8_t> argsImgPack = {clDepthBuffer, clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy, clBuffer_gz};
      for (uint8_t i = 0; i < argsImgPack.size(); i++)
         _openCL->SetArgument("packKernel", i, argsImgPack[i], true);
      _openCL->SetArgument("packKernel", argsImgPack.size(), paramsBuffer);

      std::vector<uint8_t> argsImgMerge = {clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy, clBuffer_gz, clBuffer_graph};
      for (uint8_t i = 0; i < argsImgMerge.size(); i++)
         _openCL->SetArgument("mergeKernel", i, argsImgMerge[i], true);
      _openCL->SetArgument("mergeKernel", argsImgMerge.size(), paramsBuffer);
   }

   /* Setup size for reading patch-wise kernel maps from GPU */
   cl::size_t<3> regionOutputSize;
   regionOutputSize[0] = appState.SUB_W;
   regionOutputSize[1] = appState.SUB_H;
   regionOutputSize[2] = 1;
   cl::size_t<3> origin, size;
   origin[0] = 0;
   origin[0] = 0;
   origin[0] = 0;
   size[0] = appState.INPUT_WIDTH;
   size[1] = appState.INPUT_HEIGHT;
   size[2] = 1;

   /* Output Data Buffers on CPU */
   cv::Mat debug(appState.INPUT_HEIGHT, appState.INPUT_WIDTH, CV_16UC1);
   cv::Mat output_nx(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_ny(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_nz(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_gx(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_gy(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_gz(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_graph(appState.SUB_H, appState.SUB_W, CV_8UC1);
   this->filteredDepth = cv::Mat(appState.INPUT_HEIGHT, appState.INPUT_WIDTH, CV_16UC1);

   /* Deploy the patch-packing and patch-merging kernels patch-wise */
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCL::EnqueueNDRangeKernel");
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->filterKernel, cl::NullRange, cl::NDRange(appState.FILTER_SUB_H, appState.FILTER_SUB_W),
                                                 cl::NullRange);
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->packKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->mergeKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);
   }
   ROS_INFO("Reading Images Now: (%d, %d, %d, %d, %d, %d, %d, %d)", clFilterDepth, clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy, clBuffer_gz,
            clBuffer_graph);

   /* Read the output data from OpenCL buffers into CPU buffers */
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCL::ReadImage(s)");
      // _openCL->commandQueue.enqueueReadImage(clDebug, CL_TRUE, origin, size, 0, 0, debug.data);
      _openCL->ReadImage(clFilterDepth, size, filteredDepth.data);
      _openCL->ReadImage(clBuffer_nx, regionOutputSize, output_nx.data);
      _openCL->ReadImage(clBuffer_ny, regionOutputSize, output_ny.data);
      _openCL->ReadImage(clBuffer_nz, regionOutputSize, output_nz.data);
      _openCL->ReadImage(clBuffer_gx, regionOutputSize, output_gx.data);
      _openCL->ReadImage(clBuffer_gy, regionOutputSize, output_gy.data);
      _openCL->ReadImage(clBuffer_gz, regionOutputSize, output_gz.data);
      _openCL->ReadImage(clBuffer_graph, regionOutputSize, output_graph.data);
   }
   /* Synchronize OpenCL to CPU. Block CPU until the entire OpenCL command queue has completed. */
   _openCL->commandQueue.finish();

   /* Combine the CPU buffers into single image with multiple channels */
   cv::Mat regionOutput(appState.SUB_H, appState.SUB_W, CV_32FC(6));
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCV::Merge");
      vector<cv::Mat> channels = {output_nx, output_ny, output_nz, output_gx, output_gy, output_gz};
      merge(channels, regionOutput);
      output.setRegionOutput(regionOutput);
      output.setPatchData(output_graph);
   }

   _openCL->Reset();

   ROS_INFO("Patch Graph Generated on GPU: (%d,%d,%d)", regionOutput.rows, regionOutput.cols, regionOutput.channels());

   return true;
}

void PlanarRegionCalculator::GeneratePatchGraphFromPointCloud(ApplicationState& appState, std::vector<float>& points, double inputTimestamp)
{
    MAPSENSE_PROFILE_FUNCTION();

    int ROWS = 64;
    int COLS = 1024;

    appState.KERNEL_SLIDER_LEVEL = 2;
    appState.INPUT_WIDTH = COLS;
    appState.INPUT_HEIGHT = ROWS;
    appState.PATCH_HEIGHT = appState.KERNEL_SLIDER_LEVEL;
    appState.PATCH_WIDTH = appState.KERNEL_SLIDER_LEVEL;
    appState.SUB_H = ROWS / appState.PATCH_HEIGHT;
    appState.SUB_W = COLS / appState.PATCH_WIDTH;

   /* Setup size for reading patch-wise kernel maps from GPU */
   cl::size_t<3> regionOutputSize;
   regionOutputSize[0] = appState.SUB_W;
   regionOutputSize[1] = appState.SUB_H;
   regionOutputSize[2] = 1;
   cl::size_t<3> origin, size;
   origin[0] = 0;
   origin[0] = 0;
   origin[0] = 0;
   size[0] = appState.INPUT_WIDTH;
   size[1] = appState.INPUT_HEIGHT;
   size[2] = 1;



    printf("Kernel: %d, SUB_H: %d, SUB_W: %d\n", appState.KERNEL_SLIDER_LEVEL, appState.SUB_H, appState.SUB_W);

    /*-------------------------------------------------------------------------------------------------------
     * ---------------------------------------    GPU Buffers -----------------------------------------------
     * ------------------------------------------------------------------------------------------------------
     * */
   /* Input Data GPU OpenCL Buffers */
   uint8_t paramsBuffer = CreateParameterBuffer(appState);
   uint8_t pointsBuffer = _openCL->CreateLoadBufferFloat(points.data(), points.size());

   /* Intermediate GPU OpenCL Buffers */
   uint8_t indexBuffer = _openCL->CreateReadWriteImage2D_R16(appState.INPUT_WIDTH, appState.INPUT_HEIGHT);

   /*Output Data GPU OpenCL Buffers */
   uint8_t clBuffer_nx = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_ny = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_nz = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_gx = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_gy = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_gz = _openCL->CreateReadWriteImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_graph = _openCL->CreateReadWriteImage2D_R8(appState.SUB_W, appState.SUB_H);

   ROS_INFO("Created All Input Images.");

   _openCL->SetArgument("hashKernel", 0, pointsBuffer);
   _openCL->SetArgument("hashKernel", 1, indexBuffer, true);
   _openCL->SetArgument("hashKernel", 2, paramsBuffer);
   _openCL->SetArgumentInt("hashKernel", 3, points.size());

   std::vector<uint8_t> argsImgPack = {indexBuffer, clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy, clBuffer_gz};
   for (uint8_t i = 0; i < argsImgPack.size(); i++)
      _openCL->SetArgument("packKernel", i, argsImgPack[i], true);
   _openCL->SetArgument("packKernel", argsImgPack.size(), paramsBuffer);
   _openCL->SetArgument("packKernel", argsImgPack.size() + 1, pointsBuffer);
   _openCL->SetArgumentInt("packKernel", argsImgPack.size() + 2, 1);

   std::vector<uint8_t> argsImgMerge = {clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy, clBuffer_gz, clBuffer_graph};
   for (uint8_t i = 0; i < argsImgMerge.size(); i++)
      _openCL->SetArgument("mergeKernel", i, argsImgMerge[i], true);
   _openCL->SetArgument("mergeKernel", argsImgMerge.size(), paramsBuffer);

   /*-------------------------------------------------------------------------------------------------------
    * ---------------------------------------    CPU Buffers -----------------------------------------------
    * ------------------------------------------------------------------------------------------------------
    * */
   /* Input Data CPU OpenCV Buffers */
   cv::Mat indexMat(appState.INPUT_HEIGHT, appState.INPUT_WIDTH, CV_16UC1, cv::Scalar(0));

   /* Intermediate GPU OpenCV Buffers */
   cv::Mat output_nx(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_ny(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_nz(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_gx(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_gy(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_gz(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_graph(appState.SUB_H, appState.SUB_W, CV_8UC1);


   /*-------------------------------------------------------------------------------------------------------
    * ---------------------------------------    OpenCL Kernel Calls ---------------------------------------
    * ------------------------------------------------------------------------------------------------------
    * */

    _openCL->commandQueue.enqueueNDRangeKernel(_openCL->hashKernel, cl::NullRange, cl::NDRange(appState.INPUT_HEIGHT, appState.INPUT_WIDTH),
                                               cl::NullRange);
   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->packKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);
   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->mergeKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);


   /*-------------------------------------------------------------------------------------------------------
    * ---------------------------------------    OpenCL Read Buffers ---------------------------------------
    * ------------------------------------------------------------------------------------------------------
    * */
   _openCL->ReadImage(indexBuffer, size, indexMat.data);
   _openCL->ReadImage(clBuffer_nx, regionOutputSize, output_nx.data);
   _openCL->ReadImage(clBuffer_ny, regionOutputSize, output_ny.data);
   _openCL->ReadImage(clBuffer_nz, regionOutputSize, output_nz.data);
   _openCL->ReadImage(clBuffer_gx, regionOutputSize, output_gx.data);
   _openCL->ReadImage(clBuffer_gy, regionOutputSize, output_gy.data);
   _openCL->ReadImage(clBuffer_gz, regionOutputSize, output_gz.data);
   _openCL->ReadImage(clBuffer_graph, regionOutputSize, output_graph.data);

   _openCL->commandQueue.finish();

//   AppUtils::PrintMatR16(output_gx);

//    float pitchUnit = M_PI / (2 * ROWS);
//    float yawUnit = 2 * M_PI / (COLS);
//    for (uint16_t i = 0; i < (uint16_t) (points.size() / 3); i++)
//    {
//        float x = points[i * 3];
//        float y = points[i * 3 + 1];
//        float z = points[i * 3 + 2];
//
//        float radius = sqrt(x * x + y * y);
//
//        float pitch = atan2(z, radius);
//        int pitchCount = 32 + (int) (pitch / pitchUnit);
//
//        float yaw = atan2(-y, x);
//        int yawCount = 512 + (int) (yaw / yawUnit);
//
//        if (pitchCount >= 0 && pitchCount < ROWS && yawCount >= 0 && yawCount < COLS)
//        {
//            /* countMat.at<char>(pitchCount, yawCount) */
//            indexMat.at<uint16_t>(pitchCount, yawCount) = i;
//            countMat.at<char>(pitchCount, yawCount) += 1;
//            //         printf("r:%d, c:%d, i:%d\t", pitchCount, yawCount, indexMat.at<uint16_t>(pitchCount, yawCount, 0) = i);
//        }
////        printf("\n");
//        //       printf("X: %.2lf, Y:%.2lf, Z:%.2lf, Pitch:%.2lf, Yaw:%.2lf, pc:%d, yc:%d\n", x, y, z, pitch, yaw, pitchCount, yawCount);
//    }
//
//    /* Patch Packing */
//    for (int i = 0; i < appState.SUB_H; i++)
//    {
//        for (int j = 0; j < appState.SUB_W; j++)
//        {
//            Eigen::Vector3f normal = Eigen::Vector3f::Zero();
//            Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
//            Eigen::Vector3f va = Eigen::Vector3f::Zero();
//            Eigen::Vector3f vb = Eigen::Vector3f::Zero();
//            Eigen::Vector3f vc = Eigen::Vector3f::Zero();
//            Eigen::Vector3f vd = Eigen::Vector3f::Zero();
//            uint16_t indexA = 0, indexB = 0, indexC = 0, indexD = 0;
//            uint8_t totalCount = 0, normalCount = 0;
//            for (int m = 0; m < appState.PATCH_HEIGHT - 1; m++)
//            {
//                for (int n = 0; n < appState.PATCH_WIDTH - 1; n++)
//                {
//                    indexA = indexMat.at<uint16_t>(i + m, j + n);
//                    va = Eigen::Vector3f(points[indexA * 3], points[indexA * 3 + 1], points[indexA * 3 + 2]);
//
//                    indexB = indexMat.at<uint16_t>(i + m, j + n + 1);
//                    vb = Eigen::Vector3f(points[indexB * 3], points[indexB * 3 + 1], points[indexB * 3 + 2]);
//
//                    indexC = indexMat.at<uint16_t>(i + m + 1, j + n);
//                    vc = Eigen::Vector3f(points[indexC * 3], points[indexC * 3 + 1], points[indexC * 3 + 2]);
//
//                    indexD = indexMat.at<uint16_t>(i + m + 1, j + n + 1);
//                    vd = Eigen::Vector3f(points[indexD * 3], points[indexD * 3 + 1], points[indexD * 3 + 2]);
//
//                    totalCount = (indexA != 0) + (indexB != 0) + (indexC != 0) + (indexD != 0);
//
//                    if (indexA != 0)
//                        centroid += va;
//                    if (indexB != 0)
//                        centroid += vb;
//                    if (indexC != 0)
//                        centroid += vc;
//                    if (indexD != 0)
//                        centroid += vd;
//                    if (totalCount > 0)
//                        centroid = centroid / (float) totalCount;
//
//                    if (indexA != 0 && indexB != 0 && indexC != 0)
//                    {
//                        Eigen::Vector3f nmlA = ((vb));
//                        normal += nmlA;
////                        printf("A:normal:(%.2lf,%.2lf,%.2lf)\n", nmlA.x(), nmlA.y(), nmlA.z());
//                        normalCount++;
//                    }
//                    if (indexA != 0 && indexB != 0 && indexC != 0)
//                    {
//                        Eigen::Vector3f nmlB = ((vc));
//                        normal += nmlB;
////                        printf("B:normal:(%.2lf,%.2lf,%.2lf)\n", nmlB.x(), nmlB.y(), nmlB.z());
//                        normalCount++;
//                    }
//                    if (indexA != 0 && indexB != 0 && indexC != 0)
//                    {
//                        Eigen::Vector3f nmlC = ((vd));
//                        normal += nmlC;
////                        printf("C:normal:(%.2lf,%.2lf,%.2lf)\n", nmlC.x(), nmlC.y(), nmlC.z());
//                        normalCount++;
//                    }
//                    if (indexA != 0 && indexB != 0 && indexC != 0)
//                    {
//                        Eigen::Vector3f nmlD = ((va));
//                        normal += nmlD;
////                        printf("D:normal:(%.2lf,%.2lf,%.2lf)\n", nmlD.x(), nmlD.y(), nmlD.z());
//                        normalCount++;
//                    }
//
//                    normal = normal.normalized();
//
//                    //               std::cout << va << std::endl << vb << std::endl << vc << std::endl << vd << std::endl;
//
////                    printf(
////                            "Total: %d, i:%d, j:%d, ia:%d, ib:%d, ic:%d, id:%d, ncount:%d, cA:%d, cB:%d, cC:%d, cD:%d, normal:(%.2lf,%.2lf,%.2lf), centroid:(%.2lf,%.2lf,%.2lf)\n",
////                            (uint16_t) (points.size() / 3), i, j, indexA, indexB, indexC, indexD, normalCount, countA, countB, countC, countD, normal.x(), normal.y(),
////                            normal.z(), centroid.x(), centroid.y(), centroid.z());
//                }
//            }
//
//            output_nx.at<float>(i, j) = normal.x();
//            output_ny.at<float>(i, j) = normal.y();
//            output_nz.at<float>(i, j) = normal.z();
//            output_gx.at<float>(i, j) = centroid.x();
//            output_gy.at<float>(i, j) = centroid.y();
//            output_gz.at<float>(i, j) = centroid.z();
//        }
//    }

//    /* Patch Merging */
//    for (int i = 1; i < appState.SUB_H - 1; i++)
//    {
//        for (int j = 1; j < appState.SUB_W - 1; j++)
//        {
//            uint8_t patch = 0;
//            int count = 0;
//
//            Eigen::Vector3f n_a(output_nx.at<float>(i, j), output_ny.at<float>(i, j), output_nz.at<float>(i, j));
//            Eigen::Vector3f g_a(output_gx.at<float>(i, j), output_gy.at<float>(i, j), output_gz.at<float>(i, j));
//
//            for (int k = -1; k <= 1; k += 1)
//            {
//                for (int l = -1; l <= 1; l += 1)
//                {
//                    if (!(k == 0 && l == 0))
//                    {
//                        Eigen::Vector3f n_b(output_nx.at<float>(i+k, j+l), output_ny.at<float>(i+k, j+l), output_nz.at<float>(i+k, j+l));
//                        Eigen::Vector3f g_b(output_gx.at<float>(i+k, j+l), output_gy.at<float>(i+k, j+l), output_gz.at<float>(i+k, j+l));
//
//                        if(GeomTools::CheckPatchConnection(g_a, n_a, g_b, n_b, 0.1, 0.7))
//                        {
//                            patch = (1 << count) | patch;
//                        }
//                        count++;
//                    }
//                }
//            }
//            output_graph.at<uint8_t>(i,j) = patch;
//        }
//    }

//    AppUtils::PrintMatR8(output_graph, 0, false, 0, 0);

    /* Combine the CPU buffers into single image with multiple channels */
    cv::Mat regionOutput(appState.SUB_H, appState.SUB_W, CV_32FC(6));
    vector<cv::Mat> channels = {output_nx, output_ny, output_nz, output_gx, output_gy, output_gz};
    merge(channels, regionOutput);
    output.setRegionOutput(regionOutput);
    output.setPatchData(output_graph);

    _mapFrameProcessor->generateSegmentation(output, planarRegionList); // Perform segmentation using DFS on Patch Graph on CPU to generate Planar Regions
    PlanarRegion::SetZeroId(planarRegionList);

    printf("Total Regions Found: %d\n", planarRegionList.size());

    /* DFS on Patch Tensor*/


//       AppUtils::PrintMatR16(indexMat);
//      AppUtils::PrintMatR8(countMat);
    //   AppUtils::CalculateAndPrintStatsMat(countMat);

    //   /* Combine the CPU buffers into single image with multiple channels */
    //   cv::Mat regionOutput(appState.SUB_H, appState.SUB_W, CV_32FC(6));
    //   {
    //      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCV::Merge");
    //      vector <cv::Mat> channels = {output_nx, output_ny, output_nz, output_gx, output_gy, output_gz};
    //      merge(channels, regionOutput);
    //      output.setRegionOutput(regionOutput);
    //      output.setPatchData(output_graph);
    //   }
    //
    //   _mapFrameProcessor->generateSegmentation(output, planarRegionList); // Perform segmentation using DFS on Patch Graph on CPU to generate Planar Regions
    //   PlanarRegion::SetZeroId(planarRegionList);
    //
    //   /* Planar Regions Ready To Be Published Right Here. */
    //   ROS_INFO("Number of Planar Regions: %d", planarRegionList.size());
    //   if (appState.EXPORT_REGIONS)
    //   {
    //      if (frameId % 10 == 0)
    //      {
    //         GeomTools::SaveRegions(planarRegionList, ros::package::getPath("map_sense") + "/Extras/Regions/" +
    //                                                  string(4 - to_string(frameId).length(), '0').append(to_string(frameId)) + ".txt");
    //      }
    //      frameId++;
    //   }
    //   extractRealPlanes();
}

void PlanarRegionCalculator::generateRegionsFromDepth(ApplicationState& appState, cv::Mat& depth, double inputTimestamp)
{
    MAPSENSE_PROFILE_FUNCTION();
    ROS_INFO("Generating Regions");

    _mapFrameProcessor->init(appState);

    ROS_INFO("Setting Depth");
    inputDepth = depth;
    inputTimestamp = inputTimestamp;

    ROS_INFO("Generating Patch Graph.");

    // Generate patch graph of connected patches on GPU
    bool patchGraphGenerated = GeneratePatchGraphFromDepth(appState);
    if (!patchGraphGenerated)
        return;

    _mapFrameProcessor->generateSegmentation(output, planarRegionList); // Perform segmentation using DFS on Patch Graph on CPU to generate Planar Regions
    PlanarRegion::SetZeroId(planarRegionList);

    /* Planar Regions Ready To Be Published Right Here. */
    ROS_INFO("Number of Planar Regions: %d", planarRegionList.size());
    if (appState.EXPORT_REGIONS)
    {
        if (frameId % 10 == 0)
        {
            GeomTools::SaveRegions(planarRegionList, ros::package::getPath("map_sense") + "/Extras/Regions/" +
                                                     string(4 - to_string(frameId).length(), '0').append(to_string(frameId)) + ".txt");
        }
        frameId++;
    }
    //   extractRealPlanes();
}

bool PlanarRegionCalculator::generatePatchGraphFromStereo(ApplicationState& appState)
{
   MAPSENSE_PROFILE_FUNCTION();
   ROS_INFO("Generating Patch Graph on GPU: Color:[%d,%d]", inputColor.cols, inputColor.rows, output.getRegionOutput().cols);

   if (inputColor.rows <= 0 || inputColor.cols <= 0 || inputColor.dims <= 0)
   {
      ROS_WARN("Color image width, height, or dimensions are zero! Skipping Frame.");
      return false;
   }

   this->app = appState;
   uint8_t paramsBuffer = CreateParameterBuffer(appState);

   cv::Mat colorMat;
   colorMat = inputColor.clone();

   /* Input Data OpenCL Buffers */
   uint8_t *colorBuffer = reinterpret_cast<uint8_t *>(colorMat.data);
   uint8_t clColor = _openCL->CreateLoadReadOnlyImage2D_RGBA8(colorBuffer, appState.INPUT_WIDTH, appState.INPUT_HEIGHT);

   /* Output Data OpenCL Buffers */
   uint8_t clFilterImage = _openCL->CreateReadWriteImage2D_RGBA8(appState.INPUT_WIDTH, appState.INPUT_HEIGHT);
   uint8_t clBuffer_graph = _openCL->CreateReadWriteImage2D_R8(appState.SUB_W, appState.SUB_H);

   /* Setting Kernel arguments for patch-packing kernel */
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::SetArgument(s)");

      std::vector<uint8_t> argsImgSegment = {clColor, clFilterImage, clBuffer_graph};
      for (uint8_t i = 0; i < argsImgSegment.size(); i++)
         _openCL->SetArgument("segmentKernel", i, argsImgSegment[i], true);
      _openCL->SetArgument("segmentKernel", argsImgSegment.size(), paramsBuffer);
   }

   /* Setup size for reading patch-wise kernel maps from GPU */
   cl::size_t<3> regionOutputSize;
   regionOutputSize[0] = appState.SUB_W;
   regionOutputSize[1] = appState.SUB_H;
   regionOutputSize[2] = 1;
   cl::size_t<3> origin, size;
   origin[0] = 0;
   origin[0] = 0;
   origin[0] = 0;
   size[0] = appState.INPUT_WIDTH;
   size[1] = appState.INPUT_HEIGHT;
   size[2] = 1;

   /* Output Data Buffers on CPU */
   cv::Mat output_color(appState.INPUT_HEIGHT, appState.INPUT_WIDTH, CV_8UC3);
   cv::Mat output_graph(appState.SUB_H, appState.SUB_W, CV_8UC1);

   /* Deploy the patch-packing and patch-merging kernels patch-wise */
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCL::EnqueueNDRangeKernel");
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->filterKernel, cl::NullRange, cl::NDRange(appState.FILTER_SUB_H, appState.FILTER_SUB_W),
                                                 cl::NullRange);
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->packKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->mergeKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);
   }

   /* Read the output data from OpenCL buffers into CPU buffers */
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCL::ReadImage(s)");
      // _openCL->commandQueue.enqueueReadImage(clDebug, CL_TRUE, origin, size, 0, 0, debug.data);
      _openCL->ReadImage(clFilterImage, size, filteredDepth.data);
      _openCL->ReadImage(clBuffer_graph, regionOutputSize, output_graph.data);
   }
   /* Synchronize OpenCL to CPU. Block CPU until the entire OpenCL command queue has completed. */
   _openCL->commandQueue.finish();

   /* Combine the CPU buffers into single image with multiple channels */
   //   Mat regionOutput(appState.SUB_H, appState.SUB_W, CV_32FC(6));
   //   {
   //      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCV::Merge");
   //      vector <Mat> channels = {output_nx, output_ny, output_nz, output_gx, output_gy, output_gz};
   //      merge(channels, regionOutput);
   //      output.setRegionOutput(regionOutput);
   //      output.setPatchData(output_graph);
   //   }
   //   ROS_INFO("Patch Graph Generated on GPU: (%d,%d,%d)", regionOutput.rows, regionOutput.cols, regionOutput.channels());

   _openCL->Reset();

   return true;
}

void PlanarRegionCalculator::onMouse(int event, int x, int y, int flags, void *userdata)
{
   MapFrame out = *((MapFrame *) userdata);
   if (event == cv::EVENT_MOUSEMOVE)
   {
      ROS_INFO("[%d,%d]:", y / 8, x / 8);
      ROS_INFO("%hu ", out.getPatchData().at<uint8_t>(y / 8, x / 8));
      cv::Vec6f patch = out.getRegionOutput().at<cv::Vec6f>(y / 8, x / 8);
      ROS_INFO("Center:(%.3lf, %.3lf, %.3lf), Normal:(%.3lf, %.3lf, %.3lf)\n", patch[3], patch[4], patch[5], patch[0], patch[1], patch[2]);
   }
}

void logPlanarRegions(vector<shared_ptr<PlanarRegion>> planarRegionList)
{
   for (int i = 0; i < planarRegionList.size(); i++)
   {
      ROS_INFO("ID:(%d) Center:(%.2f,%.2f,%.2f) Normal:(%.2f,%.2f,%.2f)", planarRegionList[i]->getId(), planarRegionList[i]->GetCenter().x(),
               planarRegionList[i]->GetCenter().y(), planarRegionList[i]->GetCenter().z(), planarRegionList[i]->GetNormal().x(),
               planarRegionList[i]->GetNormal().y(), planarRegionList[i]->GetNormal().z());
   }
}

void PlanarRegionCalculator::generateRegionsFromStereo(ApplicationState& appState)
{
   MAPSENSE_PROFILE_FUNCTION();

   bool patchGraphGenerated = GeneratePatchGraphFromDepth(appState);
   if (!patchGraphGenerated)
      return;

   _mapFrameProcessor->init(appState);
   _mapFrameProcessor->generateSegmentation(output, planarRegionList); // Perform segmentation using DFS on Patch Graph on CPU to generate Planar Regions

   PlanarRegion::SetZeroId(planarRegionList);
}

map_sense::RawGPUPlanarRegionList PlanarRegionCalculator::publishRegions()
{
   MAPSENSE_PROFILE_FUNCTION();
   ROS_INFO("Publishing Regions");

   if (planarRegionList.size() > 0)
   {
      for (int i = 0; i < planarRegionList.size(); i++)
      {
         map_sense::RawGPUPlanarRegion region;
         region.numOfPatches = planarRegionList[i]->getNumPatches();
         region.id = planarRegionList[i]->getId();
         region.normal = geometry_msgs::Vector3();
         region.normal.x = static_cast<double>(planarRegionList[i]->GetNormal().x());
         region.normal.y = static_cast<double>(planarRegionList[i]->GetNormal().y());
         region.normal.z = static_cast<double>(planarRegionList[i]->GetNormal().z());
         region.centroid = geometry_msgs::Point();
         region.centroid.x = static_cast<double>(planarRegionList[i]->GetCenter().x());
         region.centroid.y = static_cast<double>(planarRegionList[i]->GetCenter().y());
         region.centroid.z = static_cast<double>(planarRegionList[i]->GetCenter().z());

         for (int j = 0; j < planarRegionList[i]->getVertices().size(); j++)
         {
            Eigen::Vector3f vertex = planarRegionList[i]->getVertices()[j];
            geometry_msgs::Point point;
            point.x = static_cast<double>(vertex.x());
            point.y = static_cast<double>(vertex.y());
            point.z = static_cast<double>(vertex.z());
            region.vertices.emplace_back(point);
         }
         _planarRegionsToPublish.regions.emplace_back(region);
      }
      _planarRegionsToPublish.numOfRegions = planarRegionList.size();
      _planarRegionsToPublish.header.stamp.fromSec(inputTimestamp);
      return _planarRegionsToPublish;
      ROS_INFO("Published Regions");
   }
}

void PlanarRegionCalculator::LoadRegions(std::string path, std::vector<std::string>& fileNames, int index)
{
   planarRegionList.clear();
   GeomTools::LoadRegions(index, planarRegionList, path, fileNames);
}

bool PlanarRegionCalculator::RenderEnabled()
{
   return _render;
}
