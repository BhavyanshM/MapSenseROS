//
// Created by quantum on 3/3/21.
//

#ifndef GEOMTOOLS_H
#define GEOMTOOLS_H

#include "Eigen/Dense"

#include "MapsenseHeaders.h"
#include "PlanarRegion.h"

typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> BoolDynamicMatrix;

class AppUtils;

class GeomTools
{
   public:
      static void CanvasBoundaryDFS(uint16_t x, uint16_t y, BoolDynamicMatrix& canvas, BoolDynamicMatrix& visited, vector<Eigen::Vector2f>& concaveHull, Eigen::Vector2f start, AppUtils& appUtils, float scale);

      static vector<Eigen::Vector2f> CanvasApproximateConcaveHull(vector<Eigen::Vector2f> points, uint16_t windowHeight, uint16_t windowWidth);

      static Eigen::Matrix3f GetRotationFromAngleApproximations(Eigen::Vector3f eulerAngles);

      static Eigen::Vector3f GetProjectedPoint(Eigen::Vector4f plane, Eigen::Vector3f point);

      static vector<Eigen::Vector2f> GrahamScanConvexHull(vector<Eigen::Vector2f> points);

      static void GetParametricCurve(vector<Eigen::Vector2f> points, uint8_t m, Eigen::MatrixXf& params);

      static void CompressPointSetLinear(shared_ptr<PlanarRegion> region);

      static void LoadRegions(int frameId, vector<shared_ptr<PlanarRegion>>& regions, string directory, vector<string> files);

      static void SaveRegions(vector<shared_ptr<PlanarRegion>> regions, string fileName);

      static void TransformRegions(vector<shared_ptr<PlanarRegion>>& regions, RigidBodyTransform transform);

      static void TransformRegions(vector<shared_ptr<PlanarRegion>>& regions, Eigen::Vector3d translation, Eigen::Matrix3d rotation);

      static void LoadPoseStamped(ifstream& poseFile, Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

};

#endif //GEOMTOOLS_H
