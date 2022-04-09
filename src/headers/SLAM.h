//
// Created by quantum on 3/18/22.
//

#ifndef FACTOR_GRAPH_NODE_SLAM_H
#define FACTOR_GRAPH_NODE_SLAM_H

#include "FactorGraphHandler.h"
#include "RigidBodyTransform.h"
#include "Plane3D.h"

class SLAM
{
   public:
      SLAM ();
      void LoadPlanarSLAM(const std::string& filepath);

      void LandmarkUpdate(const std::unordered_map<int, Plane3D>& planes, int poseId);

      void PoseUpdate(const RigidBodyTransform& odometry, int poseId);

      FactorGraphHandler& GetFactorGraphHandler() {return _fgh;}

      void SLAMTest();

      void GetResultPlanes(PlaneSet3D& resultSet);

      void GetResultPose(RigidBodyTransform& transform, int poseId);

      void GetResultPoses(std::vector<RigidBodyTransform>& poses);


   private:
      FactorGraphHandler _fgh;

      RigidBodyTransform _sensorToMapTransform;

      std::vector<uint64_t> _landmarkKeys, _poseKeys;

      PlaneSet3D _planarMap;

      int _currentPoseId = 0;
};

#endif //FACTOR_GRAPH_NODE_SLAM_H
