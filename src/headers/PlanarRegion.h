#ifndef SRC_PLANARREGION_H
#define SRC_PLANARREGION_H

#include <Eigen/Dense>
#include "RegionRing.h"
#include "RigidBodyTransform.h"
#include "KDTree.h"
#include "MapsenseHeaders.h"

using namespace Eigen;
using namespace std;

class PlanarRegion
{
   private:
      Vector3f normal;
      Vector3f center;
      vector<Vector3f> patchCentroids;
      vector<Vector2f> planarPatchCentroids;
      RigidBodyTransform transformToWorldFrame;

      vector<Vector2i> leafPatches;
      KDTree tree;
      bool normalCalculated = false;
      bool centroidCalculated = false;
      int numPatches;
      int id;
      int poseId = 0;
      int numOfMeasurements = 1;

   public:
      vector<Vector3f> boundaryVertices;

      void SubSampleBoundary(int skip);

      void ComputeBoundaryVerticesPlanar();

      void ComputeBoundaryVertices3D(vector<Vector2f> points2D);

      void RetainLinearApproximation();

      void RetainConvexHull();

      int GetNumOfMeasurements() const;

      void SetNumOfMeasurements(int numOfMeasurements);

      int GetPoseId() const;

      void setPoseId(int poseId);

      Vector3f GetPCANormal();

      Vector3f getMeanCenter();

      Vector3f GetMeanNormal();

      PlanarRegion(int id);

      void AddPatch(Vector3f normal, Vector3f center);

      void insertBoundaryVertex(Vector3f vertex);

      void insertLeafPatch(Vector2i pos);

      void GetClockWise2D(vector<Vector2f>& points);

      void SortOrderClockwise();

      vector<Vector3f> getBoundaryVertices();

      int GetNumOfBoundaryVertices();

      Vector3f GetNormal();

      Vector3f GetCenter();

      vector<Vector3f> getVertices();

      vector<Vector2i> getLeafPatches();

      int getNumPatches();

      int getId();

      void setId(int id);

      void SetNormal(const Vector3f& normal);

      void SetCenter(const Vector3f& center);

      void WriteToFile(ofstream& file);

      vector<shared_ptr<RegionRing>> rings;

      void transform(RigidBodyTransform transform);

      void transform(Vector3d translation, Matrix3d rotation);

      void CopyAndTransform(shared_ptr<PlanarRegion>& planarRegionToPack, RigidBodyTransform transform);

      string toString();

      void ProjectToPlane(Vector4f plane);

      void SetToUnitSquare();

      static void PrintRegionList(const vector<shared_ptr<PlanarRegion>>& regionList, const std::string& name);

      static void SetZeroId( vector<shared_ptr<PlanarRegion>>& regionList);
};

#endif //SRC_PLANARREGION_H
