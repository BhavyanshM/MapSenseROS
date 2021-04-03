//
// Created by quantum on 2/24/21.
//

#include "SLAMApplication.h"

SLAMApplication::SLAMApplication(const Arguments& arguments) : MagnumApplication(arguments)
{
   init(arguments);
   _mesher.generateRegionLineMesh(_mapper.regions, previousRegionEdges, 1, _sensor, _drawables);
   //   generateRegionLineMesh(_mapper.latestRegions, regionEdges, 2);
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
   _mapper.getFileNames(dirPath);
   _mapper.loadRegions(frameIndex, _mapper.regions);
   _mapper.loadRegions(frameIndex + SKIP_REGIONS, _mapper.latestRegions);

   _mapper.fgSLAM.addPriorPoseFactor(Pose3().identity());
   _mapper.updateFactorGraphLandmarks(_mapper.regions, 1);

   _mapper.fgSLAM.initPoseValue(Pose3().identity());
   for (int i = 0; i < _mapper.regions.size(); i++)
   {
      shared_ptr<PlanarRegion> region = _mapper.regions[i];
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

void SLAMApplication::tickEvent()
{
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
//      case KeyEvent::Key::R:
//         auto start = high_resolution_clock::now();
//
//         slamUpdate();
//
//         auto end = high_resolution_clock::now();
//         auto duration = duration_cast<microseconds>(end - start).count();
//
//         cout << "Registration Took: " << duration / 1000.0f << " ms" << endl;
//
//         printPlanarRegions(_mapper.mapRegions, "MapRegions");
//         _mesher.generateRegionLineMesh(_mapper.mapRegions, previousRegionEdges, 1, _sensor, _drawables);
//
//         _mapper.fgSLAM.getPoses(_mapper.poses);
//
//         _mesher.generatePoseMesh(_mapper.poses, poseAxes, _sensor, _drawables);
//
//         if (frameIndex < _mapper.files.size() - SKIP_REGIONS)
//         {
//            frameIndex += SKIP_REGIONS;
//         }
//
//         /* Load previous and current regions. Separated by SKIP_REGIONS. */
//         _mapper.loadRegions(frameIndex - SKIP_REGIONS, _mapper.regions);
//         _mapper.loadRegions(frameIndex, _mapper.latestRegions);
//
//         //         _mesher.generateRegionLineMesh(_mapper.latestRegions, regionEdges, 2);
//         //         _mapper.matchPlanarRegionsToMap(_mapper.latestRegions);
//         //         _mesher.generateMatchLineMesh(_mapper, matchingEdges);
//         break;

      case KeyEvent::Key::Right:
         if (frameIndex < _mapper.files.size() - SKIP_REGIONS)
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
      case KeyEvent::Key::N:

         _mapper.matchPlanarRegionsToMap(_mapper.latestRegions);
         _mapper.registerRegions();

         /* Transform and copy the latest planar regions from current sensor frame to map frame. */
         vector<shared_ptr<PlanarRegion>> transformedRegions;
         _mapper.transformAndCopyLatestRegions(_mapper._sensorToMapTransform.getMatrix().block<3, 1>(0, 3),
                                               _mapper._sensorToMapTransform.getMatrix().block<3, 3>(0, 0), transformedRegions);

         _mesher.generateRegionLineMesh(transformedRegions, regionEdges, frameIndex, _sensor, _drawables);

         //         _mesher.generatePoseMesh(_mapper.poses, poseAxes, _sensor, _drawables);

         /* Load previous and current regions. Separated by SKIP_REGIONS. */
         if (frameIndex < _mapper.files.size() - SKIP_REGIONS)
            frameIndex += SKIP_REGIONS;
         _mapper.loadRegions(frameIndex - SKIP_REGIONS, _mapper.regions);
         _mapper.loadRegions(frameIndex, _mapper.latestRegions);

         //         _mesher.generateMatchLineMesh(_mapper, matchingEdges, _sensor, _drawables);

         break;


   }
   if (event.key() == KeyEvent::Key::Space)
   {
   }
   if (event.key() == KeyEvent::Key::Left || event.key() == KeyEvent::Key::Right)
   {
      _mapper.loadRegions(frameIndex - SKIP_REGIONS, _mapper.regions);
      _mapper.loadRegions(frameIndex, _mapper.latestRegions);

      _mesher.generateRegionLineMesh(_mapper.regions, previousRegionEdges, 1, _sensor, _drawables);
      _mesher.generateRegionLineMesh(_mapper.latestRegions, regionEdges, 2, _sensor, _drawables);

      _mapper.matchPlanarRegionsToMap(_mapper.latestRegions);
      _mesher.generateMatchLineMesh(_mapper, matchingEdges, _sensor, _drawables);
   }
}

void SLAMApplication::slamUpdate()
{
   /* Match the previous and latest regions to copy ids and generate match indices. */
   _mapper.matchPlanarRegionsToMap(_mapper.latestRegions);

   /* Calculate odometry between previous and latest poses. */
   _mapper.registerRegions();

   /* Create SE(3) matrix for odometry transform. */
   RigidBodyTransform iTj(_mapper.eulerAnglesToReference, _mapper.translationToReference);
   iTj.getInverseTransform(_mapper._sensorPoseRelative);

   /* Insert the latest landmarks and pose constraints into the Factor Graph for SLAM. */
   int currentPoseId = _mapper.updateFactorGraphPoses();
   _mapper.updateFactorGraphLandmarks(_mapper.latestRegions, currentPoseId);

   /* Initialize poses and landmarks with map frame values. */
   _mapper.initFactorGraphState();

   /* Optimize the Factor Graph and get Results. */
   _mapper.fgSLAM.optimize();

   _mapper.mergeLatestRegions();
   _mapper.updateMapRegionsWithSLAM();
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
      for (int i = 0; i < args.size(); i++)
      {
         if (args[i] == "--match-dist")
            app._mapper.MATCH_DIST_THRESHOLD = stof(args[i + 1]);
         if (args[i] == "--match-dot")
            app._mapper.MATCH_ANGULAR_THRESHOLD = stof(args[i + 1]);
         if (args[i] == "--match-vert")
            app._mapper.MATCH_PERCENT_VERTEX_THRESHOLD = stoi(args[i + 1]);
      }
      cout << "DIST:" << app._mapper.MATCH_DIST_THRESHOLD << endl;
      cout << " ANGLE:" << app._mapper.MATCH_ANGULAR_THRESHOLD << endl;
      cout << " VERT:" << app._mapper.MATCH_PERCENT_VERTEX_THRESHOLD << endl;
      return app.exec();
   }
}