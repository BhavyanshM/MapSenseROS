//
// Created by quantum on 2/24/21.
//

#include "SLAMApplication.h"

SLAMApplication::SLAMApplication(const Arguments& arguments) : MagnumApplication(arguments)
{
   this->init(arguments);
   generateRegionLineMesh(this->mapper.regions, previousRegionEdges, 1);
   generateRegionLineMesh(this->mapper.latestRegions, regionEdges, 2);

   this->mapper.matchPlanarRegionsToMap(this->mapper.latestRegions);
   generateMatchLineMesh(mapper, matchingEdges);
}

void SLAMApplication::init(const Arguments& arguments)
{
   string dirPath;
   std::vector<std::string> args(arguments.argv, arguments.argv + arguments.argc);
   for (int i = 0; i < arguments.argc; i++)
   {
      if (args[i] == "--regions-dir")
      {
         dirPath = args[i + 1];
      }
   }
   this->mapper.getFileNames(dirPath);
   this->mapper.loadRegions(frameIndex, this->mapper.regions);
   this->mapper.loadRegions(frameIndex + SKIP_REGIONS, this->mapper.latestRegions);

   this->fgSLAM.addPriorPoseFactor(Pose3().identity());
   updateFactorGraphLandmarks(this->mapper.regions);

   this->fgSLAM.initPoseValue(Pose3().identity());
   for (int i = 0; i < this->mapper.regions.size(); i++)
   {
      shared_ptr<PlanarRegion> region = this->mapper.regions[i];
      Eigen::Vector4d plane;
      plane << region->getNormal().cast<double>(), (double) -region->getNormal().dot(region->getCenter());
      this->fgSLAM.initOrientedPlaneLandmarkValue(region->getId(), plane);
   }
}

void SLAMApplication::draw()
{
}

void clear(vector<Object3D *>& objects)
{
   for (int i = 0; i < objects.size(); i++)
   {
      delete objects[i];
   }
   objects.clear();
}

void SLAMApplication::tickEvent()
{
}

void SLAMApplication::generateMatchLineMesh(PlanarRegionMapHandler mapper, vector<Object3D *>& edges)
{
   clear(edges);
   for (int i = 0; i < mapper.matches.size(); i++)
   {
      Vector3f first = this->mapper.regions[mapper.matches[i].first]->getCenter();
      Vector3f second = this->mapper.latestRegions[mapper.matches[i].second]->getCenter();
      Object3D& matchEdge = _sensor->addChild<Object3D>();
      edges.emplace_back(&matchEdge);
      new RedCubeDrawable{matchEdge, &_drawables, Magnum::Primitives::line3D({first.x(), first.y(), first.z()}, {second.x(), second.y(), second.z()}),
                          {1.0, 1.0, 1.0}};
   }
}

void SLAMApplication::generateRegionLineMesh(vector<shared_ptr<PlanarRegion>> planarRegionList, vector<Object3D *>& edges, int color)
{
   clear(edges);

   for (int i = 0; i < planarRegionList.size(); i++)
   {
      shared_ptr<PlanarRegion> planarRegion = planarRegionList[i];
      vector<Vector3f> vertices = planarRegion->getVertices();
      for (int j = SKIP_EDGES; j < vertices.size(); j += SKIP_EDGES)
      {
         Object3D& edge = _sensor->addChild<Object3D>();
         Vector3f prevPoint = vertices[j - SKIP_EDGES];
         Vector3f curPoint = vertices[j];
         edges.emplace_back(&edge);
         new RedCubeDrawable{edge, &_drawables,
                             Magnum::Primitives::line3D({prevPoint.x(), prevPoint.y(), prevPoint.z()}, {curPoint.x(), curPoint.y(), curPoint.z()}),
                             {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f, (color * 113 % 255) / 255.0f}};
         //                             {(planarRegion->getId() * 123 % 255) / 255.0f, (planarRegion->getId() * 161 % 255) / 255.0f,
         //                              (planarRegion->getId() * 113 % 255) / 255.0f}};
      }
      Object3D& regionOrigin = _sensor->addChild<Object3D>();
      regionOrigin.translateLocal({planarRegion->getCenter().x(), planarRegion->getCenter().y(), planarRegion->getCenter().z()});
      regionOrigin.scaleLocal({0.002, 0.002, 0.002});
      new RedCubeDrawable{regionOrigin, &_drawables, Magnum::Primitives::cubeSolid(),
                          {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f, (color * 113 % 255) / 255.0f}};
      edges.emplace_back(&regionOrigin);
   }
}

void printPlanarRegions(vector<shared_ptr<PlanarRegion>> regionsToPrint, string name)
{
   printf("PlanarRegions List: %s\n", name.c_str());
   for (int i = 0; i < regionsToPrint.size(); i++)
   {
      cout << regionsToPrint[i]->toString() << endl;
   }
}

void SLAMApplication::keyPressEvent(KeyEvent& event)
{
   switch (event.key())
   {
      case KeyEvent::Key::Right:
         if (frameIndex < this->mapper.files.size() - SKIP_REGIONS)
         {
            frameIndex += SKIP_REGIONS;
         }
         break;
      case KeyEvent::Key::Left:
         if (frameIndex > 1)
         {
            frameIndex -= SKIP_REGIONS;
         }
         break;
      case KeyEvent::Key::O:
         //         for(int i = 0; i<regionEdges.size(); i++){
         //            regionEdges[i]->transformLocal(Matrix4::rotationX(Rad{5.0_degf}));
         //         }
         this->mapper.transformLatestRegions(Vector3d(0, 0, 0), Vector3d(0.1, 0, 0));
         generateRegionLineMesh(this->mapper.latestRegions, regionEdges, 2);
         this->mapper.matchPlanarRegionsToMap(this->mapper.latestRegions);
         generateMatchLineMesh(mapper, matchingEdges);
         break;
      case KeyEvent::Key::R:
         auto start = high_resolution_clock::now();


         /* Match the previous and latest regions to copy ids and generate match indices. */
         this->mapper.matchPlanarRegionsToMap(this->mapper.latestRegions);

         /* Calculate odometry between previous and latest poses. */
         this->mapper.registerRegions();

         /* Create SE(3) matrix for odometry transform. */
         GeomTools::getInverseTransform(this->mapper.eulerAnglesToReference, this->mapper.translationToReference, this->mapper.sensorPoseRelative);

         /* Insert the latest landmarks and pose constraints into the Factor Graph for SLAM. */
         updateFactorGraphPoses();
         updateFactorGraphLandmarks(this->mapper.latestRegions);

         /* Initialize poses and landmarks with map frame values. */
         initFactorGraph();

         /* Optimize the Factor Graph and get Results. */
         this->fgSLAM.optimize();

         printPlanarRegions(this->mapper.regions, "Regions");
         printPlanarRegions(this->mapper.latestRegions, "LatestRegions");

         this->fgSLAM.getFactorGraph().print("Factor Graph:\n");
         this->fgSLAM.getInitial().print("Initial Estimates:\n");
         this->fgSLAM.getResults().print("Final Estimates:\n");

         auto end = high_resolution_clock::now();
         auto duration = duration_cast<microseconds>(end - start).count();

         cout << "Registration Took: " << duration / 1000.0f << " ms" << endl;

         generateRegionLineMesh(this->mapper.latestRegions, regionEdges, 2);
         this->mapper.matchPlanarRegionsToMap(this->mapper.latestRegions);
         generateMatchLineMesh(mapper, matchingEdges);
         break;
   }
   if (event.key() == KeyEvent::Key::Space)
   {
   }
   if (event.key() == KeyEvent::Key::Left || event.key() == KeyEvent::Key::Right)
   {
      this->mapper.loadRegions(frameIndex - SKIP_REGIONS, this->mapper.regions);
      this->mapper.loadRegions(frameIndex, this->mapper.latestRegions);

      printPlanarRegions(this->mapper.regions, "Regions");
      printPlanarRegions(this->mapper.latestRegions, "LatestRegions");

      generateRegionLineMesh(this->mapper.regions, previousRegionEdges, 1);
      generateRegionLineMesh(this->mapper.latestRegions, regionEdges, 2);

      this->mapper.matchPlanarRegionsToMap(this->mapper.latestRegions);
      generateMatchLineMesh(mapper, matchingEdges);
   }
}

void SLAMApplication::updateFactorGraphLandmarks(vector<shared_ptr<PlanarRegion>>& regionsToInsert)
{
   for (int i = 0; i < regionsToInsert.size(); i++)
   {
      shared_ptr<PlanarRegion> region = regionsToInsert[i];
      Eigen::Vector4d plane;
      plane << region->getNormal().cast<double>(), (double) -region->getNormal().dot(region->getCenter());
      region->setId(fgSLAM.addOrientedPlaneLandmarkFactor(plane, region->getId()));
   }
}

void SLAMApplication::updateFactorGraphPoses()
{
   fgSLAM.addOdometryFactor(this->mapper.sensorPoseRelative);
}

int main(int argc, char **argv)
{
   std::vector<std::string> args(argv, argv + argc);

   bool done = false;
   for (int i = 0; i < argc; i++)
   {
      if (args[i] == "--test")
      {
         done = true;
         PlanarRegionMapTester tester;
         tester.runTests(argc, argv);
      }
   }
   if (!done)
   {
      SLAMApplication app({argc, argv});
      return app.exec();
   }
}

void SLAMApplication::initFactorGraph()
{
   /* Update total transform from map frame to current sensor pose. Required for initial value for current pose. */
   this->mapper.mapToSensorTransform *= this->mapper.sensorPoseRelative;

   /* Calculate inverse transform from current sensor pose to map frame. Required for initial value for landmarks observed in current pose. */

   Matrix4d sensorToMapTransform = Matrix4d::Identity();
   sensorToMapTransform.block<3, 3>(0, 0) = this->mapper.mapToSensorTransform.block<3, 3>(0, 0).transpose();
   sensorToMapTransform.block<3, 1>(0, 3) =
         -this->mapper.mapToSensorTransform.block<3, 3>(0, 0).transpose() * this->mapper.mapToSensorTransform.block<3, 1>(0, 3);

   /* Transform and copy the latest planar regions from current sensor frame to map frame. */
   vector<shared_ptr<PlanarRegion>> transformedRegions;
   this->mapper.transformAndCopyLatestRegions(sensorToMapTransform.block<3, 1>(0, 3), sensorToMapTransform.block<3, 3>(0, 0), transformedRegions);

   this->fgSLAM.initPoseValue(this->mapper.mapToSensorTransform);
   for (auto region : transformedRegions)
   {
      Eigen::Vector4d plane;
      plane << region->getNormal().cast<double>(), (double) -region->getNormal().dot(region->getCenter());
      this->fgSLAM.initOrientedPlaneLandmarkValue(region->getId(), plane);
   }
}