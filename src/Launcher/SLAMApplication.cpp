//
// Created by quantum on 2/24/21.
//

#include "SLAMApplication.h"

SLAMApplication::SLAMApplication(const Arguments& arguments) : MagnumApplication(arguments)
{
   this->init(arguments);
   generateRegionLineMesh(this->_mapper.regions, previousRegionEdges, 1);
   //   generateRegionLineMesh(this->_mapper.latestRegions, regionEdges, 2);
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
   this->_mapper.getFileNames(dirPath);
   this->_mapper.loadRegions(frameIndex, this->_mapper.regions);
   this->_mapper.loadRegions(frameIndex + SKIP_REGIONS, this->_mapper.latestRegions);

   _mapper.fgSLAM.addPriorPoseFactor(Pose3().identity());
   _mapper.updateFactorGraphLandmarks(_mapper.regions, 1);

   _mapper.fgSLAM.initPoseValue(Pose3().identity());
   for (int i = 0; i < this->_mapper.regions.size(); i++)
   {
      shared_ptr<PlanarRegion> region = this->_mapper.regions[i];
      Eigen::Vector4d plane;
      plane << region->getNormal().cast<double>(), (double) -region->getNormal().dot(region->getCenter());
      _mapper.fgSLAM.initOrientedPlaneLandmarkValue(region->getId(), plane);
      _mapper.mapRegions.emplace_back(region);
      _mapper.measuredRegions.emplace_back(region);
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
      Vector3f first = this->_mapper.regions[mapper.matches[i].first]->getCenter();
      Vector3f second = this->_mapper.latestRegions[mapper.matches[i].second]->getCenter();
      Object3D& matchEdge = _sensor->addChild<Object3D>();
      edges.emplace_back(&matchEdge);
      new RedCubeDrawable{matchEdge, &_drawables, Magnum::Primitives::line3D({first.x(), first.y(), first.z()}, {second.x(), second.y(), second.z()}),
                          {1.0, 1.0, 1.0}};
   }
}

void SLAMApplication::generatePoseMesh(vector<MatrixXd> poses, vector<Object3D *>& objects)
{
   clear(objects);
   for (int i = 1; i < this->_mapper.fgSLAM.getPoseId(); i++)
   {
      Matrix4d sensorToMapTransform = this->_mapper.fgSLAM.getResults().at<Pose3>(Symbol('x', i)).matrix();
      Vector3d translation = sensorToMapTransform.block<3,1>(0,3);
      Object3D& axes = _sensor->addChild<Object3D>();
      axes.scale({0.1, 0.1, 0.1});
      axes.translateLocal({static_cast<float>(translation.x()), static_cast<float>(translation.y()), static_cast<float>(translation.z())});
      objects.emplace_back(&axes);
      new RedCubeDrawable{axes, &_drawables, Magnum::Primitives::axis3D(), {0.5, 0.3f, 0.6f}};
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
         if (frameIndex < this->_mapper.files.size() - SKIP_REGIONS)
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
         this->_mapper.transformLatestRegions(Vector3d(0, 0, 0), Vector3d(0.1, 0, 0));
         generateRegionLineMesh(this->_mapper.latestRegions, regionEdges, 2);
         this->_mapper.matchPlanarRegionsToMap(this->_mapper.latestRegions);
         generateMatchLineMesh(_mapper, matchingEdges);
         break;
      case KeyEvent::Key::R:
         auto start = high_resolution_clock::now();

         /* Match the previous and latest regions to copy ids and generate match indices. */
         _mapper.matchPlanarRegionsToMap(_mapper.latestRegions);

         /* Calculate odometry between previous and latest poses. */
         _mapper.registerRegions();

         /* Create SE(3) matrix for odometry transform. */
         GeomTools::getInverseTransform(_mapper.eulerAnglesToReference, _mapper.translationToReference, _mapper.sensorPoseRelative);

         /* Insert the latest landmarks and pose constraints into the Factor Graph for SLAM. */
         int currentPoseId = _mapper.updateFactorGraphPoses();
         _mapper.updateFactorGraphLandmarks(_mapper.latestRegions, currentPoseId);

         /* Initialize poses and landmarks with map frame values. */
         _mapper.initFactorGraphState();

         /* Optimize the Factor Graph and get Results. */
         _mapper.fgSLAM.optimize();

         _mapper.mergeLatestRegions();
         _mapper.updateMapRegions();


         //         printPlanarRegions(_mapper.regions, "Regions");
         //         printPlanarRegions(_mapper.latestRegions, "LatestRegions");


         auto end = high_resolution_clock::now();
         auto duration = duration_cast<microseconds>(end - start).count();

         cout << "Registration Took: " << duration / 1000.0f << " ms" << endl;

         printPlanarRegions(_mapper.mapRegions, "MapRegions");
         generateRegionLineMesh(_mapper.mapRegions, previousRegionEdges, 1);
         generatePoseMesh(this->_mapper.poses, poseAxes);

         if (frameIndex < this->_mapper.files.size() - SKIP_REGIONS)
         {
            frameIndex += SKIP_REGIONS;
         }

         /* Load previous and current regions. Separated by SKIP_REGIONS. */
         this->_mapper.loadRegions(frameIndex - SKIP_REGIONS, this->_mapper.regions);
         this->_mapper.loadRegions(frameIndex, this->_mapper.latestRegions);

         //         generateRegionLineMesh(this->_mapper.latestRegions, regionEdges, 2);
         //         this->_mapper.matchPlanarRegionsToMap(this->_mapper.latestRegions);
         //         generateMatchLineMesh(_mapper, matchingEdges);
         break;
   }
   if (event.key() == KeyEvent::Key::Space)
   {
   }
   if (event.key() == KeyEvent::Key::Left || event.key() == KeyEvent::Key::Right)
   {
      this->_mapper.loadRegions(frameIndex - SKIP_REGIONS, this->_mapper.regions);
      this->_mapper.loadRegions(frameIndex, this->_mapper.latestRegions);

      //      printPlanarRegions(this->_mapper.regions, "Regions");
      //      printPlanarRegions(this->_mapper.latestRegions, "LatestRegions");

      //      generateRegionLineMesh(this->_mapper.regions, previousRegionEdges, 1);
      //      generateRegionLineMesh(this->_mapper.latestRegions, regionEdges, 2);

      this->_mapper.matchPlanarRegionsToMap(this->_mapper.latestRegions);
      //      generateMatchLineMesh(_mapper, matchingEdges);
   }
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
         //         tester.testKDTree();
      }
   }
   if (!done)
   {
      SLAMApplication app({argc, argv});
      return app.exec();
   }
}