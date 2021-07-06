#include "PlanarRegionCalculator.h"

PlanarRegionCalculator::PlanarRegionCalculator(int argc, char **argv, NetworkManager *network, ApplicationState& app)
{
   ROS_DEBUG("Creating PlanarRegionCalculator");
   this->_dataReceiver = network;
   this->_mapFrameProcessor.init(app);
   this->inputDepth = Mat(app.INPUT_HEIGHT, app.INPUT_WIDTH, CV_16UC1);
   this->inputColor = Mat(app.INPUT_HEIGHT, app.INPUT_WIDTH, CV_8UC3);
   origin[0] = 0;
   origin[0] = 0;
   origin[0] = 0;
}

void PlanarRegionCalculator::ImGuiUpdate(ApplicationState& appState)
{
   ImGui::Text("Input:%d,%d Patch:%d,%d Level:%d", appState.INPUT_HEIGHT, appState.INPUT_WIDTH, appState.PATCH_HEIGHT, appState.PATCH_WIDTH,
               appState.KERNEL_SLIDER_LEVEL);
   ImGui::Checkbox("Generate Regions", &appState.GENERATE_REGIONS);
   ImGui::Checkbox("Filter", &appState.FILTER_SELECTED);
   ImGui::Checkbox("Early Gaussian", &appState.EARLY_GAUSSIAN_BLUR);
   ImGui::SliderInt("Gaussian Size", &appState.GAUSSIAN_SIZE, 1, 4);
   ImGui::SliderInt("Gaussian Sigma", &appState.GAUSSIAN_SIGMA, 1, 20);
   ImGui::SliderFloat("Distance Threshold", &appState.MERGE_DISTANCE_THRESHOLD, 0.0f, 0.1f);
   ImGui::SliderFloat("Angular Threshold", &appState.MERGE_ANGULAR_THRESHOLD, 0.0f, 1.0f);
   ImGui::Checkbox("Components", &appState.SHOW_REGION_COMPONENTS);
   ImGui::Checkbox("Boundary", &appState.SHOW_BOUNDARIES);
   ImGui::Checkbox("Internal", &appState.SHOW_PATCHES);
   if (ImGui::Button("Hide Display"))
   {
      destroyAllWindows();
      destroyAllWindows();
   }
   ImGui::SliderFloat("Display Window Size", &appState.DISPLAY_WINDOW_SIZE, 0.1, 5.0);
   ImGui::SliderFloat("Depth Brightness", &appState.DEPTH_BRIGHTNESS, 1.0, 100.0);
   ImGui::Checkbox("Visual Debug", &appState.VISUAL_DEBUG);
   ImGui::SliderInt("Visual Debug Delay", &appState.VISUAL_DEBUG_DELAY, 1, 100);
   ImGui::Checkbox("Show Edges", &appState.SHOW_REGION_EDGES);
   ImGui::SliderInt("Skip Edges", &appState.NUM_SKIP_EDGES, 1, 20);
   ImGui::SliderFloat("Magnum Patch Scale", &appState.MAGNUM_PATCH_SCALE, 0.001f, 0.04f);
}

void PlanarRegionCalculator::render()
{
   if (this->app.SHOW_REGION_COMPONENTS)
   {
      ROS_DEBUG("Appending Region Components");
      AppUtils::DisplayImage(_mapFrameProcessor.debug, app);
      //      appUtils.appendToDebugOutput(this->_mapFrameProcessor.debug);
   }
   if (this->app.SHOW_STEREO_LEFT)
   {
      appUtils.appendToDebugOutput(this->inputStereoLeft);
   }
   if (this->app.SHOW_STEREO_RIGHT)
   {
      appUtils.appendToDebugOutput(this->inputStereoRight);
   }
   //   appUtils.displayDebugOutput(app);
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
bool PlanarRegionCalculator::generatePatchGraph(ApplicationState appState)
{
   MAPSENSE_PROFILE_FUNCTION();
   ROS_DEBUG("Generating Patch Graph on GPU: Color:[%d,%d] Depth:[%d,%d] Output:[%d,%d]", inputColor.cols, inputColor.rows, inputDepth.cols, inputDepth.rows,
             output.getRegionOutput().cols, output.getRegionOutput().rows);

   if (inputDepth.rows <= 0 || inputDepth.cols <= 0 || inputDepth.dims <= 0)
   {
      ROS_DEBUG("Depth image width, height, or dimensions are zero! Skipping Frame.");
      return false;
   }

   this->app = appState;
   float params[] = {(float) appState.FILTER_DISPARITY_THRESHOLD, appState.MERGE_ANGULAR_THRESHOLD, appState.MERGE_DISTANCE_THRESHOLD,
                     (float) appState.PATCH_HEIGHT, (float) appState.PATCH_WIDTH, (float) appState.SUB_H, (float) appState.SUB_W, appState.DEPTH_FX,
                     appState.DEPTH_FY, appState.DEPTH_CX, appState.DEPTH_CY, (float) appState.FILTER_KERNEL_SIZE, (float) appState.FILTER_SUB_H,
                     (float) appState.FILTER_SUB_W, (float) appState.INPUT_HEIGHT, (float) appState.INPUT_WIDTH};

   ROS_DEBUG("GenerateRegions:(%d, %d, %d, %d, %d, %d) Filter:(%d,%d):%d", appState.INPUT_HEIGHT, appState.INPUT_WIDTH, appState.PATCH_HEIGHT,
             appState.PATCH_WIDTH, appState.SUB_H, appState.SUB_W, appState.FILTER_SUB_H, appState.FILTER_SUB_W, appState.FILTER_KERNEL_SIZE);

   uint8_t paramsBuffer = _openCL->CreateBufferFloat(params, sizeof(params) / sizeof(float));

   Mat depthMat, colorMat;
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCV::Clone");
      depthMat = inputDepth.clone();
      colorMat = inputColor.clone();
   }
   ROS_DEBUG("Cloned Depth and Color Successfully.");

   //   medianBlur(depthMat, depthMat, 5);

   ROS_DEBUG("Applying Gaussian Filter [GaussianSize: %d, GaussianSigma: %d]( Channels: %d, Dims: %d, Rows: %d, Cols: %d)", appState.GAUSSIAN_SIZE,
             appState.GAUSSIAN_SIGMA, depthMat.channels(), depthMat.dims, depthMat.rows, depthMat.cols);

   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCV::GaussianBlur");
      if (appState.EARLY_GAUSSIAN_BLUR)
         GaussianBlur(depthMat, depthMat, Size(appState.GAUSSIAN_SIZE * 2 + 1, appState.GAUSSIAN_SIZE * 2 + 1), appState.GAUSSIAN_SIGMA);
   }
   ROS_DEBUG("Gaussian Filter Applied Successfully.");

   /* Input Data OpenCL Buffers */
   uint16_t *depthBuffer = reinterpret_cast<uint16_t *>(depthMat.data);
   uint8_t *colorBuffer = reinterpret_cast<uint8_t *>(colorMat.data);
   uint8_t clDepth = _openCL->CreateImage2D_R16(depthBuffer, appState.INPUT_WIDTH, appState.INPUT_HEIGHT);
   uint8_t clColor = _openCL->CreateImage2D_RGBA8(colorBuffer, appState.INPUT_WIDTH, appState.INPUT_HEIGHT);

   /* Output Data OpenCL Buffers */
   ROS_DEBUG("Creating OpenCL Image2Ds now.");

   uint8_t clFilterDepth = _openCL->CreateOutputImage2D_R16(appState.INPUT_WIDTH, appState.INPUT_HEIGHT);
   uint8_t clBuffer_nx = _openCL->CreateOutputImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_ny = _openCL->CreateOutputImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_nz = _openCL->CreateOutputImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_gx = _openCL->CreateOutputImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_gy = _openCL->CreateOutputImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_gz = _openCL->CreateOutputImage2D_RFloat(appState.SUB_W, appState.SUB_H);
   uint8_t clBuffer_graph = _openCL->CreateOutputImage2D_R8(appState.SUB_W, appState.SUB_H);

   ROS_DEBUG("Created All Input Images.");

   /* Setting Kernel arguments for patch-packing kernel */
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::SetArgument(s)");
      _openCL->SetArgument("filterKernel", 0, clDepth, true);
      _openCL->SetArgument("filterKernel", 1, clColor, true);
      _openCL->SetArgument("filterKernel", 2, clFilterDepth, true);
      _openCL->SetArgument("filterKernel", 3, clBuffer_nx, true);
      _openCL->SetArgument("filterKernel", 4, paramsBuffer);

      if (appState.FILTER_SELECTED)
      {
         _openCL->SetArgument("packKernel", 0, clFilterDepth, true);
      } else
      {
         _openCL->SetArgument("packKernel", 0, clDepth, true);
      }

      _openCL->SetArgument("packKernel", 1, clBuffer_nx, true); // Output: nx
      _openCL->SetArgument("packKernel", 2, clBuffer_ny, true); // Output: ny
      _openCL->SetArgument("packKernel", 3, clBuffer_nz, true); // Output: nz
      _openCL->SetArgument("packKernel", 4, clBuffer_gx, true); // Output: gx
      _openCL->SetArgument("packKernel", 5, clBuffer_gy, true); // Output: gy
      _openCL->SetArgument("packKernel", 6, clBuffer_gz, true); // Output: gz
      _openCL->SetArgument("packKernel", 7, paramsBuffer);

      _openCL->SetArgument("mergeKernel", 0, clBuffer_nx, true); // Input: nx
      _openCL->SetArgument("mergeKernel", 1, clBuffer_ny, true); // Input: ny
      _openCL->SetArgument("mergeKernel", 2, clBuffer_nz, true); // Input: nz
      _openCL->SetArgument("mergeKernel", 3, clBuffer_gx, true); // Input: gx
      _openCL->SetArgument("mergeKernel", 4, clBuffer_gy, true); // Input: gy
      _openCL->SetArgument("mergeKernel", 5, clBuffer_gz, true); // Input: gz
      _openCL->SetArgument("mergeKernel", 6, clBuffer_graph, true);  // Output: graph
      _openCL->SetArgument("mergeKernel", 7, paramsBuffer);

      // kernel.setArg(4, clDebug); /* For whenever clDebug may be required. */
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
   Mat debug(appState.INPUT_HEIGHT, appState.INPUT_WIDTH, CV_16UC1);
   Mat output_0(appState.SUB_H, appState.SUB_W, CV_32FC1);
   Mat output_1(appState.SUB_H, appState.SUB_W, CV_32FC1);
   Mat output_2(appState.SUB_H, appState.SUB_W, CV_32FC1);
   Mat output_3(appState.SUB_H, appState.SUB_W, CV_32FC1);
   Mat output_4(appState.SUB_H, appState.SUB_W, CV_32FC1);
   Mat output_5(appState.SUB_H, appState.SUB_W, CV_32FC1);
   Mat output_6(appState.SUB_H, appState.SUB_W, CV_8UC1);
   this->filteredDepth = Mat(appState.INPUT_HEIGHT, appState.INPUT_WIDTH, CV_16UC1);


   /* Deploy the patch-packing and patch-merging kernels patch-wise */
   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCL::EnqueueNDRangeKernel");
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->filterKernel, cl::NullRange, cl::NDRange(appState.FILTER_SUB_H, appState.FILTER_SUB_W),
                                                 cl::NullRange);
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->packKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);
      _openCL->commandQueue.enqueueNDRangeKernel(_openCL->mergeKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);
   }
   ROS_DEBUG("Reading Images Now: (%d, %d, %d, %d, %d, %d, %d, %d)", clFilterDepth, clBuffer_nx, clBuffer_ny, clBuffer_nz, clBuffer_gx, clBuffer_gy,
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

   ROS_DEBUG("Packaging Layers Now. ");

   /* Combine the CPU buffers into single image with multiple channels */
   Mat regionOutput(appState.SUB_H, appState.SUB_W, CV_32FC(6));
   vector <Mat> channels;

   {
      MAPSENSE_PROFILE_SCOPE("GeneratePatchGraph::OpenCV::Merge");
      channels.push_back(output_0);
      channels.push_back(output_1);
      channels.push_back(output_2);
      channels.push_back(output_3);
      channels.push_back(output_4);
      channels.push_back(output_5);
      merge(channels, regionOutput);

      output.setRegionOutput(regionOutput);
      output.setPatchData(output_6);
   }

   _openCL->Reset();

   ROS_DEBUG("Patch Graph Generated on GPU: (%d,%d,%d)", regionOutput.rows, regionOutput.cols, regionOutput.channels());

   return true;
}

void PlanarRegionCalculator::onMouse(int event, int x, int y, int flags, void *userdata)
{
   MapFrame out = *((MapFrame *) userdata);
   if (event == EVENT_MOUSEMOVE)
   {
      ROS_DEBUG("[%d,%d]:", y / 8, x / 8);
      ROS_DEBUG("%hu ", out.getPatchData().at<uint8_t>(y / 8, x / 8));
      Vec6f patch = out.getRegionOutput().at<Vec6f>(y / 8, x / 8);
      ROS_DEBUG("Center:(%.3lf, %.3lf, %.3lf), Normal:(%.3lf, %.3lf, %.3lf)\n", patch[3], patch[4], patch[5], patch[0], patch[1], patch[2]);
   }
}

void logPlanarRegions(vector <shared_ptr<PlanarRegion>> planarRegionList)
{
   for (int i = 0; i < planarRegionList.size(); i++)
   {
      ROS_DEBUG("ID:(%d) Center:(%.2f,%.2f,%.2f) Normal:(%.2f,%.2f,%.2f)", planarRegionList[i]->getId(), planarRegionList[i]->getCenter().x(),
                planarRegionList[i]->getCenter().y(), planarRegionList[i]->getCenter().z(), planarRegionList[i]->getNormal().x(),
                planarRegionList[i]->getNormal().y(), planarRegionList[i]->getNormal().z());
   }
}

void PlanarRegionCalculator::generateAndPublishRegions(ApplicationState appState)
{
   MAPSENSE_PROFILE_FUNCTION();
   ROS_DEBUG("Generating Regions");

   ImageReceiver *depthReceiver = ((ImageReceiver *) this->_dataReceiver->receivers[0]);
   depthReceiver->getData(inputDepth, appState, inputTimestamp);

   // Generate patch graph of connected patches on GPU
   _mapFrameProcessor.init(appState);
   bool patchGraphGenerated = generatePatchGraph(appState);
   if (!patchGraphGenerated)
      return;

   _mapFrameProcessor.generateSegmentation(output, planarRegionList); // Perform segmentation using DFS on Patch Graph on CPU to generate Planar Regions
   PlanarRegion::SetZeroId(planarRegionList);

   /* Planar Regions Ready To Be Published Right Here. */
   ROS_DEBUG("Number of Planar Regions: %d", planarRegionList.size());
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
   publishRegions(planarRegionList);
}

void PlanarRegionCalculator::publishRegions(vector <shared_ptr<PlanarRegion>> rawRegionList)
{
   MAPSENSE_PROFILE_FUNCTION();
   ROS_DEBUG("Publishing Regions");
   map_sense::RawGPUPlanarRegionList planarRegionsToPublish;
   if (rawRegionList.size() > 0)
   {
      for (int i = 0; i < rawRegionList.size(); i++)
      {
         map_sense::RawGPUPlanarRegion region;
         region.numOfPatches = rawRegionList[i]->getNumPatches();
         region.id = rawRegionList[i]->getId();
         region.normal = geometry_msgs::Vector3();
         region.normal.x = static_cast<double>(rawRegionList[i]->getNormal().x());
         region.normal.y = static_cast<double>(rawRegionList[i]->getNormal().y());
         region.normal.z = static_cast<double>(rawRegionList[i]->getNormal().z());
         region.centroid = geometry_msgs::Point();
         region.centroid.x = static_cast<double>(rawRegionList[i]->getCenter().x());
         region.centroid.y = static_cast<double>(rawRegionList[i]->getCenter().y());
         region.centroid.z = static_cast<double>(rawRegionList[i]->getCenter().z());

         for (int j = 0; j < rawRegionList[i]->getVertices().size(); j++)
         {
            Vector3f vertex = rawRegionList[i]->getVertices()[j];
            geometry_msgs::Point point;
            point.x = static_cast<double>(vertex.x());
            point.y = static_cast<double>(vertex.y());
            point.z = static_cast<double>(vertex.z());
            region.vertices.emplace_back(point);
         }
         planarRegionsToPublish.regions.emplace_back(region);
      }
      planarRegionsToPublish.numOfRegions = rawRegionList.size();
      planarRegionsToPublish.header.stamp.fromSec(inputTimestamp);
      _dataReceiver->planarRegionPub.publish(planarRegionsToPublish);
      ROS_DEBUG("Published Regions");
   }
}


