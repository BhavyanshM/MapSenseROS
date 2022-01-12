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

uint8_t PlanarRegionCalculator::loadParameters(const ApplicationState& app)
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
bool PlanarRegionCalculator::generatePatchGraph(ApplicationState& appState)
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
   uint8_t paramsBuffer = loadParameters(appState);

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
   cv::Mat output_0(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_1(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_2(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_3(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_4(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_5(appState.SUB_H, appState.SUB_W, CV_32FC1);
   cv::Mat output_6(appState.SUB_H, appState.SUB_W, CV_8UC1);
   this->filteredDepth = cv::Mat(appState.INPUT_HEIGHT, appState.INPUT_WIDTH, CV_16UC1);

   /* Deploy the patch-packing and patch-merging kernels patch-wise */
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCL::EnqueueNDRangeKernel");
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->filterKernel, cl::NullRange, cl::NDRange(appState.FILTER_SUB_H, appState.FILTER_SUB_W),
                                                 cl::NullRange);
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->packKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->mergeKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);
   }
   ROS_INFO("Reading Images Now: (%d, %d, %d, %d, %d, %d, %d, %d)", clFilterDepth, clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy,
             clBuffer_gz, clBuffer_graph);

   /* Read the output data from OpenCL buffers into CPU buffers */
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCL::ReadImage(s)");
      // _openCL->commandQueue.enqueueReadImage(clDebug, CL_TRUE, origin, size, 0, 0, debug.data);
      _openCL->ReadImage(clFilterDepth, size, filteredDepth.data);
      _openCL->ReadImage(clBuffer_nx, regionOutputSize, output_0.data);
      _openCL->ReadImage(clBuffer_ny, regionOutputSize, output_1.data);
      _openCL->ReadImage(clBuffer_nz, regionOutputSize, output_2.data);
      _openCL->ReadImage(clBuffer_gx, regionOutputSize, output_3.data);
      _openCL->ReadImage(clBuffer_gy, regionOutputSize, output_4.data);
      _openCL->ReadImage(clBuffer_gz, regionOutputSize, output_5.data);
      _openCL->ReadImage(clBuffer_graph, regionOutputSize, output_6.data);
   }
   /* Synchronize OpenCL to CPU. Block CPU until the entire OpenCL command queue has completed. */
   _openCL->commandQueue.finish();

   /* Combine the CPU buffers into single image with multiple channels */
   cv::Mat regionOutput(appState.SUB_H, appState.SUB_W, CV_32FC(6));
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCV::Merge");
      vector <cv::Mat> channels = {output_0, output_1, output_2, output_3, output_4, output_5};
      merge(channels, regionOutput);
      output.setRegionOutput(regionOutput);
      output.setPatchData(output_6);
   }

   _openCL->Reset();

   ROS_INFO("Patch Graph Generated on GPU: (%d,%d,%d)", regionOutput.rows, regionOutput.cols, regionOutput.channels());

   return true;
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
   uint8_t paramsBuffer = loadParameters(appState);

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
   cv::Mat output_6(appState.SUB_H, appState.SUB_W, CV_8UC1);

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
      _openCL->ReadImage(clBuffer_graph, regionOutputSize, output_6.data);
   }
   /* Synchronize OpenCL to CPU. Block CPU until the entire OpenCL command queue has completed. */
   _openCL->commandQueue.finish();

   /* Combine the CPU buffers into single image with multiple channels */
//   Mat regionOutput(appState.SUB_H, appState.SUB_W, CV_32FC(6));
//   {
//      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCV::Merge");
//      vector <Mat> channels = {output_0, output_1, output_2, output_3, output_4, output_5};
//      merge(channels, regionOutput);
//      output.setRegionOutput(regionOutput);
//      output.setPatchData(output_6);
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

void logPlanarRegions(vector <shared_ptr<PlanarRegion>> planarRegionList)
{
   for (int i = 0; i < planarRegionList.size(); i++)
   {
      ROS_INFO("ID:(%d) Center:(%.2f,%.2f,%.2f) Normal:(%.2f,%.2f,%.2f)", planarRegionList[i]->getId(), planarRegionList[i]->getCenter().x(),
                planarRegionList[i]->getCenter().y(), planarRegionList[i]->getCenter().z(), planarRegionList[i]->getNormal().x(),
                planarRegionList[i]->getNormal().y(), planarRegionList[i]->getNormal().z());
   }
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
   bool patchGraphGenerated = generatePatchGraph(appState);
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
         GeomTools::saveRegions(planarRegionList, ros::package::getPath("map_sense") + "/Extras/Regions/" +
                                                  string(4 - to_string(frameId).length(), '0').append(to_string(frameId)) + ".txt");
      }
      frameId++;
   }
   //   extractRealPlanes();
}

void PlanarRegionCalculator::generateRegionsFromStereo(ApplicationState& appState)
{
   MAPSENSE_PROFILE_FUNCTION();

   bool patchGraphGenerated = generatePatchGraph(appState);
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
         region.normal.x = static_cast<double>(planarRegionList[i]->getNormal().x());
         region.normal.y = static_cast<double>(planarRegionList[i]->getNormal().y());
         region.normal.z = static_cast<double>(planarRegionList[i]->getNormal().z());
         region.centroid = geometry_msgs::Point();
         region.centroid.x = static_cast<double>(planarRegionList[i]->getCenter().x());
         region.centroid.y = static_cast<double>(planarRegionList[i]->getCenter().y());
         region.centroid.z = static_cast<double>(planarRegionList[i]->getCenter().z());

         for (int j = 0; j < planarRegionList[i]->getVertices().size(); j++)
         {
            Vector3f vertex = planarRegionList[i]->getVertices()[j];
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


