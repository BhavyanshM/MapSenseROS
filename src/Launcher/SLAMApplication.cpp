#include "SLAMApplication.h"

SLAMApplication::SLAMApplication(const Arguments& arguments) : MagnumApplication(arguments), _mesher(&_drawables)
{
   init(arguments);
}

void SLAMApplication::init(const Arguments& arguments)
{
   _mesher.generateRegionLineMesh(_mapper.regions, previousRegionEdges, 1, _world);
   string dirPath;
   std::vector<std::string> args(arguments.argv, arguments.argv + arguments.argc);
   for (int i = 0; i < arguments.argc; i++)
   {
      if (args[i] == "--regions-dir")
      {
         dirPath = args[i + 1];
      }
   }
   AppUtils::getFileNames(dirPath, _mapper.files);
   _mapper.setDirectory(dirPath);
   GeomTools::loadRegions(frameIndex, _mapper.regions, _mapper.directory, _mapper.files);
   GeomTools::loadRegions(frameIndex + SKIP_REGIONS, _mapper.latestRegions, _mapper.directory, _mapper.files);



   printf("Creating Prior Factors\n");
   printf("PoseId:(%d)", _mapper.fgSLAM->getPoseId());
   _mapper.fgSLAM->addPriorPoseFactor(Pose3().identity(), 1);
   _mapper.insertOrientedPlaneFactors(1);

   _mapper.fgSLAM->setPoseInitialValue(1, Pose3().identity());
   for (int i = 0; i < _mapper.regions.size(); i++)
   {
      shared_ptr<PlanarRegion> region = _mapper.regions[i];
      Eigen::Vector4d plane;
      plane << region->getNormal().cast<double>(), (double) -region->getNormal().dot(region->getCenter());
      _mapper.fgSLAM->setOrientedPlaneInitialValue(region->getId(), plane);
      _mapper.mapRegions.emplace_back(region);
      _mapper.measuredRegions.emplace_back(region);
   }

//   _mesher.generateRegionLineMesh(_mapper.regions, previousRegionEdges, 1, _world);
   _mesher.generateRegionLineMesh(_mapper.latestRegions, latestRegionEdges, 2, _world);
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

      case KeyEvent::Key::N: // Next

         auto start = high_resolution_clock::now();

         /* Match the previous and latest regions to copy ids and generate match indices. Calculate odometry between previous and latest poses. */
         _mapper.matchPlanarRegionsToMap(_mapper.latestRegions);
         _mapper.registerRegionsPointToPlane(1);

         /* Insert the local landmark measurements and odometry constraints into the Factor Graph for SLAM. */
         int currentPoseId = _mapper.fgSLAM->addOdometryFactor(MatrixXd(_mapper._sensorPoseRelative.getMatrix()));
         _mapper.insertOrientedPlaneFactors(currentPoseId);

         /* Transform and copy the latest planar regions from current sensor frame to map frame.  */
         vector<shared_ptr<PlanarRegion>> regionsInMapFrame;
         GeomTools::transformAndCopyRegions(_mapper.latestRegions, regionsInMapFrame, _mapper._sensorToMapTransform);

         if (_mapper.FACTOR_GRAPH)
         {
            /* Initialize poses and landmarks with map frame values. */
            _mapper.setOrientedPlaneInitialValues();
            //            _mapper.fgSLAM.addPriorPoseFactor(Pose3(_mapper._sensorToMapTransform.getInverse().getMatrix()), currentPoseId);

            /* Optimize the Factor Graph and get Results. */
            _mapper.optimize();
            //_mapper.mergeLatestRegions();
            _mapper.extractFactorGraphLandmarks();

            auto end = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(end - start).count();
            printf("SLAM Update Took: %.2lf ms\n", duration/(float)1000);

            for(shared_ptr<PlanarRegion> region : _mapper.mapRegions) GeomTools::compressPointSetLinear(region);

            _mesher.generateRegionLineMesh(_mapper.mapRegions, latestRegionEdges, frameIndex, _world);
            _mapper.fgSLAM->getPoses(_mapper.poses);

            if (_mapper.ISAM2)
               _mapper.fgSLAM->clearISAM2();
         } else
         {
            _mapper.poses.emplace_back(RigidBodyTransform(_mapper._sensorToMapTransform));
            _mesher.generateRegionLineMesh(regionsInMapFrame, latestRegionEdges, frameIndex, _world);
         }

         _mesher.generatePoseMesh(_mapper.poses, poseAxes, _world);

         /* Load previous and current regions. Separated by SKIP_REGIONS. */
         if (frameIndex < _mapper.files.size() - SKIP_REGIONS)
         {
            frameIndex += SKIP_REGIONS;
         }
         _mapper.regions = _mapper.latestRegions;
         GeomTools::loadRegions(frameIndex, _mapper.latestRegions, _mapper.directory, _mapper.files);

         //         _mesher.generateMatchLineMesh(_mapper.matches, _mapper.regions, _mapper.latestRegions, matchingEdges, _world);
         break;
   }

   if (event.key() == KeyEvent::Key::P){ // Project Plane
      Vector4f plane;
      plane << _mapper.regions[0]->getNormal(), -_mapper.regions[0]->getNormal().dot(_mapper.regions[0]->getCenter());
      _mapper.latestRegions[0]->projectToPlane(plane);

      cout << "Reached" << endl;
      _mapper.latestRegions[0]->retainLinearApproximation();
      _mesher.generateRegionLineMesh(_mapper.latestRegions, latestRegionEdges, 3, _world, false);


   }

   if (event.key() == KeyEvent::Key::Space)
   {
   }
   if (event.key() == KeyEvent::Key::Left || event.key() == KeyEvent::Key::Right)
   {
      GeomTools::loadRegions(frameIndex - SKIP_REGIONS, _mapper.regions, _mapper.directory, _mapper.files);
      GeomTools::loadRegions(frameIndex, _mapper.latestRegions, _mapper.directory, _mapper.files);

      _mesher.generateRegionLineMesh(_mapper.regions, previousRegionEdges, 1, _world);
      _mesher.generateRegionLineMesh(_mapper.latestRegions, latestRegionEdges, 2, _world);

      _mapper.matchPlanarRegionsToMap(_mapper.latestRegions);
      _mesher.generateMatchLineMesh(_mapper.matches, _mapper.regions, _mapper.latestRegions, matchingEdges, _world);
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
      if(args[i] == "--sfm")
      {
         StructureFromMotion sfm(args[i + 1]);
      }
   }
   if (!done)
   {
      SLAMApplication app({argc, argv});
      for (int i = 0; i < args.size(); i++)
      {
         if (args[i] == "--match-dist")
         {
            app._mapper.MATCH_DIST_THRESHOLD = stof(args[i + 1]);
            cout << "DIST:" << app._mapper.MATCH_DIST_THRESHOLD << "\t";
         }
         if (args[i] == "--match-dot")
         {
            app._mapper.MATCH_ANGULAR_THRESHOLD = stof(args[i + 1]);
            cout << "ANGLE:" << app._mapper.MATCH_ANGULAR_THRESHOLD << "\t";
         }
         if (args[i] == "--match-vert")
         {
            app._mapper.MATCH_PERCENT_VERTEX_THRESHOLD = stoi(args[i + 1]);
            cout << "VERT:" << app._mapper.MATCH_PERCENT_VERTEX_THRESHOLD << "\t";
         }
         if (args[i] == "--factor-graph")
         {
            app._mapper.FACTOR_GRAPH = true;
            cout << "Factor Graph: true" << endl;
         }
         if (args[i] == "--isam")
         {
            app._mapper.ISAM2 = true;
            app._mapper.ISAM2_NUM_STEPS = stoi(args[i + 1]);
            cout << "Incremental SAM2: true \nSTEPS: " << app._mapper.ISAM2_NUM_STEPS << endl;
         }

      }
      cout << endl;
      return app.exec();
   }
}