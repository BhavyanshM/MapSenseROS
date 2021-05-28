//
// Created by quantum on 5/28/21.
//

#include "SLAMModule.h"

SLAMModule::SLAMModule(int argc, char **argv)
{
   std::vector<std::string> args(argv, argv + argc);
   for (int i = 0; i < args.size(); i++)
   {
      if (args[i] == "--match-dist")
      {
         _mapper.MATCH_DIST_THRESHOLD = stof(args[i + 1]);
         cout << "DIST:" << _mapper.MATCH_DIST_THRESHOLD << "\t";
      }
      if (args[i] == "--match-dot")
      {
         _mapper.MATCH_ANGULAR_THRESHOLD = stof(args[i + 1]);
         cout << "ANGLE:" << _mapper.MATCH_ANGULAR_THRESHOLD << "\t";
      }
      if (args[i] == "--match-vert")
      {
         _mapper.MATCH_PERCENT_VERTEX_THRESHOLD = stoi(args[i + 1]);
         cout << "VERT:" << _mapper.MATCH_PERCENT_VERTEX_THRESHOLD << "\t";
      }
      if (args[i] == "--factor-graph")
      {
         _mapper.FACTOR_GRAPH = true;
         cout << "Factor Graph: true" << endl;
      }
      if (args[i] == "--isam")
      {
         _mapper.ISAM2 = true;
         _mapper.ISAM2_NUM_STEPS = stoi(args[i + 1]);
         cout << "Incremental SAM2: true \nSTEPS: " << _mapper.ISAM2_NUM_STEPS << endl;
      }
   }
}

void SLAMModule::init()
{
   _mapper.fgSLAM.addPriorPoseFactor(Pose3().identity(), 1);
   _mapper.fgSLAM.initPoseValue(Pose3().identity());
}

void SLAMModule::slamUpdate(vector<shared_ptr<PlanarRegion>> latestRegions)
{
   auto start = high_resolution_clock::now();

   _mapper.latestRegions = latestRegions;

   /* Match the previous and latest regions to copy ids and generate match indices. Calculate odometry between previous and latest poses. */
   _mapper.matchPlanarRegionsToMap(_mapper.latestRegions);
   _mapper.registerRegionsPointToPlane();

   /* Insert the local landmark measurements and odometry constraints into the Factor Graph for SLAM. */
   int currentPoseId = _mapper.updateFactorGraphPoses(_mapper._sensorPoseRelative);
   _mapper.updateFactorGraphLandmarks(_mapper.latestRegions, currentPoseId);

   /* Transform and copy the latest planar regions from current sensor frame to map frame.  */
   vector<shared_ptr<PlanarRegion>> regionsInMapFrame;
   _mapper.transformAndCopyLatestRegions(_mapper._sensorToMapTransform, regionsInMapFrame);

   if (_mapper.FACTOR_GRAPH)
   {
      /* Initialize poses and landmarks with map frame values. */
      _mapper.initFactorGraphState(_mapper._sensorToMapTransform.getInverse(), regionsInMapFrame);
      //            _mapper.fgSLAM.addPriorPoseFactor(Pose3(_mapper._sensorToMapTransform.getInverse().getMatrix()), currentPoseId);

      /* Optimize the Factor Graph and get Results. */
      _mapper.ISAM2 ? _mapper.fgSLAM.optimizeISAM2(_mapper.ISAM2_NUM_STEPS) : _mapper.fgSLAM.optimize();
      //_mapper.mergeLatestRegions();
      _mapper.updateMapRegionsWithSLAM();

      auto end = high_resolution_clock::now();
      auto duration = duration_cast<microseconds>(end - start).count();
      printf("SLAM Update Took: %.2lf ms\n", duration / (float) 1000);

      for (shared_ptr<PlanarRegion> region : _mapper.mapRegions)
         GeomTools::compressPointSetLinear(region);

      //      _mesher.generateRegionLineMesh(_mapper.mapRegions, latestRegionEdges, frameIndex, _sensor);
      _mapper.fgSLAM.getPoses(_mapper.poses);

      if (_mapper.ISAM2)
         _mapper.fgSLAM.clearISAM2();
   } else
   {
      _mapper.poses.emplace_back(RigidBodyTransform(_mapper._sensorToMapTransform));
      //      _mesher.generateRegionLineMesh(regionsInMapFrame, latestRegionEdges, frameIndex, _sensor);
   }

   //   _mesher.generatePoseMesh(_mapper.poses, poseAxes, _sensor);

   /* Load previous and current regions. Separated by SKIP_REGIONS. */
   _mapper.regions = _mapper.latestRegions;

   //   _mesher.generateMatchLineMesh(_mapper.matches, _mapper.regions, _mapper.latestRegions, matchingEdges, _sensor);

}