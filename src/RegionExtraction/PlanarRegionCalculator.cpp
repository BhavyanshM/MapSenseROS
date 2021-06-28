#include "PlanarRegionCalculator.h"

PlanarRegionCalculator::PlanarRegionCalculator(int argc, char **argv, NetworkManager *network, ApplicationState& app)
{
   ROS_INFO("Creating PlanarRegionCalculator");
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

   //   ImGui::Checkbox("Color", &appState.SHOW_INPUT_COLOR);
   //   ImGui::Checkbox("Depth", &appState.SHOW_INPUT_DEPTH);
   //   ImGui::Checkbox("Filtered", &appState.SHOW_FILTERED_DEPTH);
   //   ImGui::SameLine(180);
   //   ImGui::Checkbox("Graph", &appState.SHOW_GRAPH);
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

   //    ImGui::SliderInt("Region Boundary Diff", &appState.REGION_BOUNDARY_DIFF, 10, 40);
   //    ImGui::SliderInt("Region Min Patches", &appState.REGION_MIN_PATCHES, 4, 100);

   //    ImGui::SliderFloat("Filter Disparity Threshold", &appState.FILTER_DISPARITY_THRESHOLD, 1000, 4000);
   //    ImGui::SliderInt("Kernel Level", &appState.KERNEL_SLIDER_LEVEL, 2, 10);
   //    ImGui::SliderInt("Filter Size", &appState.FILTER_KERNEL_SIZE, 2, 10);
   //    appState.update();
}

void PlanarRegionCalculator::render()
{
   if (this->app.SHOW_REGION_COMPONENTS)
      appUtils.appendToDebugOutput(this->_mapFrameProcessor.debug);

   if (this->app.SHOW_STEREO_LEFT)
   {
      appUtils.appendToDebugOutput(this->inputStereoLeft);
   }
   if (this->app.SHOW_STEREO_RIGHT)
   {
      appUtils.appendToDebugOutput(this->inputStereoRight);
   }
   appUtils.displayDebugOutput(this->app);
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

   ROS_INFO("Generating Patch Graph on GPU: Color:[%d,%d] Depth:[%d,%d] Output:[%d,%d]", inputColor.cols, inputColor.rows, inputDepth.cols, inputDepth.rows,
             output.getRegionOutput().cols, output.getRegionOutput().rows);

   if (inputDepth.rows <= 0 || inputDepth.cols <= 0 || inputDepth.dims <= 0)
   {
      ROS_INFO("Depth image width, height, or dimensions are zero! Skipping Frame.");
      return false;
   }

   this->app = appState;
   float params[] = {(float) appState.FILTER_DISPARITY_THRESHOLD, appState.MERGE_ANGULAR_THRESHOLD, appState.MERGE_DISTANCE_THRESHOLD,
                     (float) appState.PATCH_HEIGHT, (float) appState.PATCH_WIDTH, (float) appState.SUB_H, (float) appState.SUB_W, appState.DEPTH_FX,
                     appState.DEPTH_FY, appState.DEPTH_CX, appState.DEPTH_CY, (float) appState.FILTER_KERNEL_SIZE, (float) appState.FILTER_SUB_H,
                     (float) appState.FILTER_SUB_W, (float) appState.INPUT_HEIGHT, (float) appState.INPUT_WIDTH};

   ROS_INFO("GenerateRegions:(%d, %d, %d, %d, %d, %d) Filter:(%d,%d):%d", appState.INPUT_HEIGHT, appState.INPUT_WIDTH, appState.PATCH_HEIGHT,
             appState.PATCH_WIDTH, appState.SUB_H, appState.SUB_W, appState.FILTER_SUB_H, appState.FILTER_SUB_W, appState.FILTER_KERNEL_SIZE);
   cl::Buffer paramsBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, sizeof(params), params);

   Mat depthMat = inputDepth.clone();
   Mat colorMat = inputColor.clone();

   ROS_INFO("Cloned Depth and Color Successfully.");

   //   medianBlur(depthMat, depthMat, 5);


   ROS_INFO("Applying Gaussian Filter [GaussianSize: %d, GaussianSigma: %d]( Channels: %d, Dims: %d, Rows: %d, Cols: %d)", appState.GAUSSIAN_SIZE,
             appState.GAUSSIAN_SIGMA, depthMat.channels(), depthMat.dims, depthMat.rows, depthMat.cols);
   if (appState.EARLY_GAUSSIAN_BLUR)
      GaussianBlur(depthMat, depthMat, Size(appState.GAUSSIAN_SIZE * 2 + 1, appState.GAUSSIAN_SIZE * 2 + 1), appState.GAUSSIAN_SIGMA);

   ROS_INFO("Gaussian Filter Applied Successfully.");

   /* Input Data OpenCL Buffers */
   uint16_t *depthBuffer = reinterpret_cast<uint16_t *>(depthMat.data);
   uint8_t *colorBuffer = reinterpret_cast<uint8_t *>(colorMat.data);
   cl::Image2D clDepth(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, cl::ImageFormat(CL_R, CL_UNSIGNED_INT16), appState.INPUT_WIDTH, appState.INPUT_HEIGHT,
                       0, depthBuffer);
   cl::Image2D clColor(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, cl::ImageFormat(CL_RGBA, CL_UNSIGNED_INT8), appState.INPUT_WIDTH,
                       appState.INPUT_HEIGHT, 0, colorBuffer);

   /* Output Data OpenCL Buffers */
   // cl::Image2D clDebug(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_UNSIGNED_INT16), appState.INPUT_WIDTH, appState.INPUT_HEIGHT);
   cl::Image2D clFilterDepth(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_UNSIGNED_INT16), appState.INPUT_WIDTH, appState.INPUT_HEIGHT);
   cl::Image2D clBuffer_nx(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), appState.SUB_W, appState.SUB_H);
   cl::Image2D clBuffer_ny(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), appState.SUB_W, appState.SUB_H);
   cl::Image2D clBuffer_nz(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), appState.SUB_W, appState.SUB_H);
   cl::Image2D clBuffer_gx(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), appState.SUB_W, appState.SUB_H);
   cl::Image2D clBuffer_gy(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), appState.SUB_W, appState.SUB_H);
   cl::Image2D clBuffer_gz(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_FLOAT), appState.SUB_W, appState.SUB_H);
   cl::Image2D clBuffer_graph(context, CL_MEM_READ_WRITE, cl::ImageFormat(CL_R, CL_UNSIGNED_INT8), appState.SUB_W, appState.SUB_H);

   /* Setting Kernel arguments for patch-packing kernel */
   filterKernel.setArg(0, clDepth);
   filterKernel.setArg(1, clColor);
   filterKernel.setArg(2, clFilterDepth);
   filterKernel.setArg(3, clBuffer_nx);
   filterKernel.setArg(4, paramsBuffer);

   if (appState.FILTER_SELECTED)
   {
      packKernel.setArg(0, clFilterDepth);
   } else
   {
      packKernel.setArg(0, clDepth);
   }
   packKernel.setArg(1, clBuffer_nx); // Output: nx
   packKernel.setArg(2, clBuffer_ny); // Output: ny
   packKernel.setArg(3, clBuffer_nz); // Output: nz
   packKernel.setArg(4, clBuffer_gx); // Output: gx
   packKernel.setArg(5, clBuffer_gy); // Output: gy
   packKernel.setArg(6, clBuffer_gz); // Output: gz
   packKernel.setArg(7, paramsBuffer);

   mergeKernel.setArg(0, clBuffer_nx); // Input: nx
   mergeKernel.setArg(1, clBuffer_ny); // Input: ny
   mergeKernel.setArg(2, clBuffer_nz); // Input: nz
   mergeKernel.setArg(3, clBuffer_gx); // Input: gx
   mergeKernel.setArg(4, clBuffer_gy); // Input: gy
   mergeKernel.setArg(5, clBuffer_gz); // Input: gz
   mergeKernel.setArg(6, clBuffer_graph);  // Output: graph
   mergeKernel.setArg(7, paramsBuffer);

   // kernel.setArg(4, clDebug); /* For whenever clDebug may be required. */

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
   commandQueue.enqueueNDRangeKernel(filterKernel, cl::NullRange, cl::NDRange(appState.FILTER_SUB_H, appState.FILTER_SUB_W), cl::NullRange);
   commandQueue.enqueueNDRangeKernel(packKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);
   commandQueue.enqueueNDRangeKernel(mergeKernel, cl::NullRange, cl::NDRange(appState.SUB_H, appState.SUB_W), cl::NullRange);

   ROS_INFO("FilteredDepth:(%d,%d)", filteredDepth.rows, filteredDepth.cols);
   /* Read the output data from OpenCL buffers into CPU buffers */
   // commandQueue.enqueueReadImage(clDebug, CL_TRUE, origin, size, 0, 0, debug.data);
   commandQueue.enqueueReadImage(clFilterDepth, CL_TRUE, origin, size, 0, 0, filteredDepth.data);
   commandQueue.enqueueReadImage(clBuffer_nx, CL_TRUE, origin, regionOutputSize, 0, 0, output_0.data);
   commandQueue.enqueueReadImage(clBuffer_ny, CL_TRUE, origin, regionOutputSize, 0, 0, output_1.data);
   commandQueue.enqueueReadImage(clBuffer_nz, CL_TRUE, origin, regionOutputSize, 0, 0, output_2.data);
   commandQueue.enqueueReadImage(clBuffer_gx, CL_TRUE, origin, regionOutputSize, 0, 0, output_3.data);
   commandQueue.enqueueReadImage(clBuffer_gy, CL_TRUE, origin, regionOutputSize, 0, 0, output_4.data);
   commandQueue.enqueueReadImage(clBuffer_gz, CL_TRUE, origin, regionOutputSize, 0, 0, output_5.data);
   commandQueue.enqueueReadImage(clBuffer_graph, CL_TRUE, origin, regionOutputSize, 0, 0, output_6.data);

   /* Synchronize OpenCL to CPU. Block CPU until the entire OpenCL command queue has completed. */
   commandQueue.finish();

   /* Combine the CPU buffers into single image with multiple channels */
   Mat regionOutput(appState.SUB_H, appState.SUB_W, CV_32FC(6));
   vector<Mat> channels;
   channels.push_back(output_0);
   channels.push_back(output_1);
   channels.push_back(output_2);
   channels.push_back(output_3);
   channels.push_back(output_4);
   channels.push_back(output_5);
   merge(channels, regionOutput);

   output.setRegionOutput(regionOutput);
   output.setPatchData(output_6);

   ROS_INFO("Patch Graph Generated on GPU: (%d,%d,%d)", regionOutput.rows, regionOutput.cols, regionOutput.channels());

   return true;
}

void PlanarRegionCalculator::initOpenCL(ApplicationState app)
{

   printf("Initializing OpenCL\n");

   vector<cl::Platform> all_platforms;
   cl::Platform::get(&all_platforms);

   if (all_platforms.size() == 0)
   {
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
   if (!fp)
   {
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
   //    kernel_code = "#define SIZE_X " + to_string(this->PATCH_HEIGHT) + "\n"
   //                  + "#define SIZE_Y " + to_string(this->PATCH_WIDTH)
   //                  + kernel_code;
   sources.push_back({kernel_code.c_str(), kernel_code.length()});
   cl::Program program(context, sources);
   if (program.build({default_device}) != CL_SUCCESS)
   {
      cout << " Error building: " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(default_device) << "\n";
      exit(1);
   }
   commandQueue = cl::CommandQueue(context, default_device);
   filterKernel = cl::Kernel(program, "filterKernel");
   packKernel = cl::Kernel(program, "packKernel");
   mergeKernel = cl::Kernel(program, "mergeKernel");

   printf("OpenCL Initialized Successfully\n");
}

void PlanarRegionCalculator::onMouse(int event, int x, int y, int flags, void *userdata)
{
   MapFrame out = *((MapFrame *) userdata);
   if (event == EVENT_MOUSEMOVE)
   {
      ROS_INFO("[%d,%d]:", y / 8, x / 8);
      ROS_INFO("%hu ", out.getPatchData().at<uint8_t>(y / 8, x / 8));
      Vec6f patch = out.getRegionOutput().at<Vec6f>(y / 8, x / 8);
      ROS_INFO("Center:(%.3lf, %.3lf, %.3lf), Normal:(%.3lf, %.3lf, %.3lf)\n", patch[3], patch[4], patch[5], patch[0], patch[1], patch[2]);
   }
}

void logPlanarRegions(vector<shared_ptr<PlanarRegion>> planarRegionList)
{
   for (int i = 0; i < planarRegionList.size(); i++)
   {
      ROS_INFO("ID:(%d) Center:(%.2f,%.2f,%.2f) Normal:(%.2f,%.2f,%.2f)", planarRegionList[i]->getId(), planarRegionList[i]->getCenter().x(),
                planarRegionList[i]->getCenter().y(), planarRegionList[i]->getCenter().z(), planarRegionList[i]->getNormal().x(),
                planarRegionList[i]->getNormal().y(), planarRegionList[i]->getNormal().z());
   }
}

void PlanarRegionCalculator::generateAndPublishRegions(ApplicationState appState)
{
   ROS_INFO("Generating Regions");

   ImageReceiver *depthReceiver = ((ImageReceiver *) this->_dataReceiver->receivers[0]);
   depthReceiver->getData(inputDepth, appState, inputTimestamp);

   this->_mapFrameProcessor.init(appState);
   this->generatePatchGraph(appState); // Generate patch graph of connected patches on GPU

   // Generate patch graph of connected patches on GPU
   bool patchGraphGenerated = generatePatchGraph(appState);
   if (!patchGraphGenerated)
      return;

   _mapFrameProcessor.generateSegmentation(output, planarRegionList); // Perform segmentation using DFS on Patch Graph on CPU to generate Planar Regions

   //   setMouseCallback("DebugOutput", PlanarRegionCalculator::onMouse, (void *) &output);
   //    ROS_INFO("CurrentPlanarRegions");
   //    logPlanarRegions(planarRegionList);
   //    ROS_INFO("CurrentPlanarRegions");
   //    logPlanarRegions(previousRegionList);

   PlanarRegion::SetZeroId(planarRegionList);

   /* Planar Regions Ready To Be Published Right Here. */

   publishRegions(planarRegionList);

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

   publishRegions(planarRegionList);
}


void PlanarRegionCalculator::publishRegions(vector<shared_ptr<PlanarRegion>> rawRegionList)
{
   ROS_INFO("Publishing Regions");
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
      ROS_INFO("Published Regions");
   }
}


