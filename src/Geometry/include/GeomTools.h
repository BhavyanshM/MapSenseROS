//
// Created by quantum on 3/3/21.
//

#ifndef GEOMTOOLS_H
#define GEOMTOOLS_H

#include "Eigen/Dense"
#include <stack>
#include "PlanarRegion.h"
#include "AppUtils.h"

using namespace Eigen;

typedef Eigen::Matrix<bool, Dynamic, Dynamic> BoolDynamicMatrix;

class GeomTools
{
   public:
      static void canvasBoundaryDFS(uint16_t x, uint16_t y, BoolDynamicMatrix& canvas, BoolDynamicMatrix& visited, vector<Vector2f>& concaveHull, Vector2f start, AppUtils& appUtils, float scale);

      static vector<Vector2f> canvasApproximateConcaveHull(vector<Vector2f> points, uint16_t windowHeight, uint16_t windowWidth);

      static Matrix3f getRotationFromAngleApproximations(Vector3f eulerAngles);

      static Vector3f getProjectedPoint(Vector4f plane, Vector3f point);

      static vector<Vector2f> grahamScanConvexHull(vector<Vector2f> points);

      static void getParametricCurve(vector<Vector2f> points, uint8_t m, MatrixXf& params);

      static void compressPointSetLinear(shared_ptr<PlanarRegion> region);

      static void loadRegions(int frameId, vector<shared_ptr<PlanarRegion>>& regions, string directory, vector<string> files);

      static void transformRegions(vector<shared_ptr<PlanarRegion>>& regions, RigidBodyTransform transform);

      static void transformRegions(vector<shared_ptr<PlanarRegion>>& regions, Vector3d translation, Matrix3d rotation);

      static void loadPoseStamped(ifstream& poseFile, Vector3d& position, Quaterniond& orientation);
};

#endif //GEOMTOOLS_H
