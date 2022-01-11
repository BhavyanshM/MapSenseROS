#include "MapFrameProcessor.h"

MapFrameProcessor::MapFrameProcessor(ApplicationState& app) : app(app) {}

void MapFrameProcessor::init(ApplicationState& app)
{
   MAPSENSE_PROFILE_FUNCTION();
   ROS_INFO("Initializing MapFrameProcessor");
   this->app = app;
   this->debug = cv::Mat(app.INPUT_HEIGHT, app.INPUT_WIDTH, CV_8UC3);
   this->visited = MatrixXi(app.SUB_H, app.SUB_W).setZero();
   this->boundary = MatrixXi(app.SUB_H, app.SUB_W).setZero();
   this->region = MatrixXi(app.SUB_H, app.SUB_W).setZero();
   ROS_INFO("MapFrameProcessor Initialized.");
}

void MapFrameProcessor::generateSegmentation(MapFrame inputFrame, vector<shared_ptr<PlanarRegion>>& planarRegionList)
{
   MAPSENSE_PROFILE_FUNCTION();
   ROS_INFO("Starting DFS for Segmentation\n");
   this->app = app;
   this->frame = inputFrame;

   /* For initial development only. Delete all old previous regions before inserting new ones. Old and new regions should be fused instead. */
   uint8_t components = 0;
   {
      MAPSENSE_PROFILE_SCOPE("GenerateSegmentation::DFS");
      planarRegionList.clear();
      visited.setZero();
      region.setZero();
      boundary.setZero();
      debug = cv::Scalar(0);
      for (uint16_t i = 0; i < app.SUB_H; i++)
      {
         for (uint16_t j = 0; j < app.SUB_W; j++)
         {
            uint8_t patch = inputFrame.patchData.at<uint8_t>(i, j);
            if (!visited(i, j) && patch == 255)
            {
               int num = 0;
               shared_ptr <PlanarRegion> planarRegion = make_shared<PlanarRegion>(components);
               dfs(i, j, components, num, debug, planarRegion);
               if (num > app.REGION_MIN_PATCHES && num - planarRegion->getNumOfBoundaryVertices() > app.REGION_BOUNDARY_DIFF)
               {
                  planarRegionList.emplace_back(planarRegion);
                  components++;
               }
            }
         }
      }
   }

   ROS_INFO("DFS Generated %d Regions\n", components);

   /* Extract Region Boundary Indices. */
   visited.setZero();

   findBoundaryAndHoles(planarRegionList);

   /* Grow Region Boundary. */
   growRegionBoundary(planarRegionList);
   ROS_INFO("Regions Grown Manually");

}

void MapFrameProcessor::findBoundaryAndHoles(vector<shared_ptr<PlanarRegion>>& planarRegionList)
{
   MAPSENSE_PROFILE_FUNCTION();
   tbb::parallel_for(tbb::blocked_range<int>(0, planarRegionList.size()), [&](const tbb::blocked_range<int>& r)
   {
      for (int i = r.begin(); i < r.end(); i++)
      {
         int components = 0;
         vector<Vector2i> leafPatches = planarRegionList[i]->getLeafPatches();
         for (int j = 0; j < leafPatches.size(); j++)
         {
            int num = 0;
            shared_ptr<RegionRing> regionRing = make_shared<RegionRing>(components);
            boundary_dfs(leafPatches[j].x(), leafPatches[j].y(), planarRegionList[i]->getId(), components, num, debug, regionRing);
            if (num > 3)
            {
               planarRegionList[i]->rings.emplace_back(regionRing);
               components++;
            }
         }
         auto comp = [](const shared_ptr<RegionRing> a, const shared_ptr<RegionRing> b)
         {
            return a->getNumOfVertices() > b->getNumOfVertices();
         };
         sort(planarRegionList[i]->rings.begin(), planarRegionList[i]->rings.end(), comp);
         //      vector<Vector2i> ring = planarRegionList[i]->rings[0]->boundaryIndices;
         //      for (int j = 0; j < ring.size(); j++)
         //      {
         //         Vec6f patch = this->frame.getRegionOutput().at<Vec6f>(ring[j].x(), ring[j].y());
         //         planarRegionList[i]->insertBoundaryVertex(Vector3f(patch[3], patch[4], patch[5]));
         //      }
      }
   });

//   for (int i = 0; i < planarRegionList.size(); i++)
//   {
//      int components = 0;
//      vector<Vector2i> leafPatches = planarRegionList[i]->getLeafPatches();
//      for (int j = 0; j < leafPatches.size(); j++)
//      {
//         int num = 0;
//         shared_ptr<RegionRing> regionRing = make_shared<RegionRing>(components);
//         boundary_dfs(leafPatches[j].x(), leafPatches[j].y(), planarRegionList[i]->getId(), components, num, debug, regionRing);
//         if (num > 3)
//         {
//            planarRegionList[i]->rings.emplace_back(regionRing);
//            components++;
//         }
//      }
//      auto comp = [](const shared_ptr<RegionRing> a, const shared_ptr<RegionRing> b)
//      {
//         return a->getNumOfVertices() > b->getNumOfVertices();
//      };
//      sort(planarRegionList[i]->rings.begin(), planarRegionList[i]->rings.end(), comp);
////      vector<Vector2i> ring = planarRegionList[i]->rings[0]->boundaryIndices;
////      for (int j = 0; j < ring.size(); j++)
////      {
////         Vec6f patch = this->frame.getRegionOutput().at<Vec6f>(ring[j].x(), ring[j].y());
////         planarRegionList[i]->insertBoundaryVertex(Vector3f(patch[3], patch[4], patch[5]));
////      }
//   }
}



void MapFrameProcessor::growRegionBoundary(vector<shared_ptr<PlanarRegion>>& regions)
{
   MAPSENSE_PROFILE_FUNCTION();
   tbb::parallel_for(tbb::blocked_range<int>(0, regions.size()), [&](const tbb::blocked_range<int>& r){
      for(int i = r.begin(); i<r.end(); i++)
      {
         if(regions[i]->rings.size() <= 0)
         {
            ROS_WARN("Region Boundary Size is Zero!");
            ROS_ASSERT_MSG(regions[i]->rings.size() == 0, "Region Boundary Size is Zero!");
         }

         Vector3f center = regions[i]->getCenter();
         vector<Vector2i> ring = regions[i]->rings[0]->boundaryIndices;
         for (int j = 0; j < ring.size(); j++)
         {
            cv::Vec6f patch = this->frame.getRegionOutput().at<cv::Vec6f>(ring[j].x(), ring[j].y());
            Vector3f vertex(patch[3], patch[4], patch[5]);
            regions[i]->insertBoundaryVertex(vertex + (vertex - center).normalized() * this->app.REGION_GROWTH_FACTOR);
         }
      }
   });

   //   for(int i = 0; i<regions.size(); i++)
   //   {
   //      if(regions[i]->rings.size() <= 0)
   //      {
   //         ROS_WARN("Region Boundary Size is Zero!");
   //         ROS_ASSERT_MSG(regions[i]->rings.size() == 0, "Region Boundary Size is Zero!");
   //      }
   //
   //      Vector3f center = regions[i]->getCenter();
   //      vector<Vector2i> ring = regions[i]->rings[0]->boundaryIndices;
   //      for (int j = 0; j < ring.size(); j++)
   //      {
   //         Vec6f patch = this->frame.getRegionOutput().at<Vec6f>(ring[j].x(), ring[j].y());
   //         Vector3f vertex(patch[3], patch[4], patch[5]);
   //         regions[i]->insertBoundaryVertex(vertex + (vertex - center).normalized() * this->app.REGION_GROWTH_FACTOR);
   //      }
   //   }
}


void MapFrameProcessor::printPatchGraph()
{
   printf("DEBUGGER:(%d,%d), AppState:(%d,%d)\n", app.SUB_H, app.SUB_W, app.SUB_H, app.SUB_W);
   for (int i = 0; i < app.SUB_H; i++)
   {
      for (int j = 0; j < app.SUB_W; j++)
      {
         uint8_t current = this->frame.patchData.at<uint8_t>(i, j);
         if (current == 255)
         {
            printf("1 ");
         } else
         {
            printf("0 ");
         }
      }
      printf("\n");
   }
}

void MapFrameProcessor::displayDebugger(int delay)
{
   imshow("DebugOutput", this->debug);
   int code = cv::waitKeyEx(delay);
   if (code == 113)
   {
      this->app.VISUAL_DEBUG = false;
   }
}


///////////////////////////////////////////////////////////////////
/* ----------------- RECURSIVE FUNCTIONS BELOW THIS POINT -------*/
///////////////////////////////////////////////////////////////////

void MapFrameProcessor::boundary_dfs(int x, int y, int regionId, int component, int& num, cv::Mat& debug, shared_ptr<RegionRing> regionRing)
{
   if (visited(x, y))
      return;

   num++;
   visited(x, y) = 1;
   regionRing->insertBoundaryIndex(Vector2i(x,y));
   if (app.SHOW_BOUNDARIES)
      circle(debug, cv::Point((y) * app.PATCH_HEIGHT, (x) * app.PATCH_WIDTH), 2,
             cv::Scalar((component + 1) * 130 % 255, (component + 1) * 227 % 255, (component + 1) * 332 % 255), -1);
   for (int i = 0; i < 8; i++)
   {
      if (x + adx[i] < app.SUB_H - 1 && x + adx[i] > 1 && y + ady[i] < app.SUB_W - 1 && y + ady[i] > 1)
      {
         if (boundary(x + adx[i], y + ady[i]) == 1 && regionId == region(x + adx[i], y + ady[i]))
         {
            boundary_dfs(x + adx[i], y + ady[i], regionId, component, num, debug, regionRing);
         }
      }
   }
   if (app.VISUAL_DEBUG)
      displayDebugger(app.VISUAL_DEBUG_DELAY);
}

void MapFrameProcessor::dfs(uint16_t x, uint16_t y, uint8_t component, int& num, cv::Mat& debug, shared_ptr<PlanarRegion> planarRegion)
{
   if (visited(x, y))
      return;

   num++;
   visited(x, y) = 1;
   region(x, y) = component;
   cv::Vec6f patch = this->frame.getRegionOutput().at<cv::Vec6f>(x, y);
   planarRegion->addPatch(Vector3f(patch[0], patch[1], patch[2]), Vector3f(patch[3], patch[4], patch[5]));
   if (app.SHOW_PATCHES)
      circle(debug, cv::Point((y) * app.PATCH_HEIGHT, (x) * app.PATCH_WIDTH), 2,
             cv::Scalar((component + 1) * 231 % 255, (component + 1) * 123 % 255, (component + 1) * 312 % 255), -1);

   int count = 0;
   for (int i = 0; i < 8; i++)
   {
      if (x + adx[i] < app.SUB_H - 1 && x + adx[i] > 1 && y + ady[i] < app.SUB_W - 1 && y + ady[i] > 1)
      {
         uint8_t newPatch = this->frame.patchData.at<uint8_t>((x + adx[i]), (y + ady[i]));
         if (newPatch == 255)
         {
            count++;
            dfs(x + adx[i], y + ady[i], component, num, debug, planarRegion);
         }
      }
   }
   if (count != 8)
   {
      boundary(x, y) = 1;
      planarRegion->insertLeafPatch(Vector2i(x, y));
      if (app.SHOW_BOUNDARIES)
         circle(debug, cv::Point((y) * app.PATCH_HEIGHT, (x) * app.PATCH_WIDTH), 2, cv::Scalar(255, 255, 255), -1);
   }
   if (app.VISUAL_DEBUG)
      displayDebugger(app.VISUAL_DEBUG_DELAY);
}
