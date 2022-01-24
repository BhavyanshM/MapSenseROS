#ifndef SRC_PLANARREGION_H
#define SRC_PLANARREGION_H

#include <Eigen/Dense>
#include "RegionRing.h"
#include "RigidBodyTransform.h"
#include "KDTree.h"
#include "MapsenseHeaders.h"

using namespace std;

class PlanarRegion
{
   private:
      Eigen::Vector3f normal;
      Eigen::Vector3f center;
      vector<Eigen::Vector3f> patchCentroids;
      vector<Eigen::Vector2f> planarPatchCentroids;
      RigidBodyTransform transformToWorldFrame;

      vector<Eigen::Vector2i> leafPatches;
      KDTree tree;
      bool normalCalculated = false;
      bool centroidCalculated = false;
      int numPatches;
      int id;
      int poseId = 0;
      int numOfMeasurements = 1;

   public:
      vector<Eigen::Vector3f> boundaryVertices;

      void SubSampleBoundary(int skip);

      void ComputeBoundaryVerticesPlanar();

      void ComputeBoundaryVertices3D(vector<Eigen::Vector2f> points2D);

      void RetainLinearApproximation();

      void RetainConvexHull();

      int GetNumOfMeasurements() const;

      void SetNumOfMeasurements(int numOfMeasurements);

      int GetPoseId() const;

      void setPoseId(int poseId);

      Eigen::Vector3f GetPCANormal();

      Eigen::Vector3f getMeanCenter();

      Eigen::Vector3f GetMeanNormal();

      PlanarRegion(int id);

      void AddPatch(Eigen::Vector3f normal, Eigen::Vector3f center);

      void insertBoundaryVertex(Eigen::Vector3f vertex);

      void insertLeafPatch(Eigen::Vector2i pos);

      void GetClockWise2D(vector<Eigen::Vector2f>& points);

      void SortOrderClockwise();

      vector<Eigen::Vector3f> getBoundaryVertices();

      int GetNumOfBoundaryVertices();

      Eigen::Vector3f GetNormal();

      Eigen::Vector3f GetCenter();

      vector<Eigen::Vector3f> getVertices();

      vector<Eigen::Vector2i> getLeafPatches();

      int getNumPatches();

      int getId();

      void setId(int id);

      void SetNormal(const Eigen::Vector3f& normal);

      void SetCenter(const Eigen::Vector3f& center);

      void WriteToFile(ofstream& file);

      vector<shared_ptr<RegionRing>> rings;

      void transform(RigidBodyTransform transform);

      void transform(Eigen::Vector3d translation, Eigen::Matrix3d rotation);

      void CopyAndTransform(shared_ptr<PlanarRegion>& planarRegionToPack, RigidBodyTransform transform);

      string toString();

      void ProjectToPlane(Eigen::Vector4f plane);

      void SetToUnitSquare();

      static void PrintRegionList(const vector<shared_ptr<PlanarRegion>>& regionList, const std::string& name);

      static void SetZeroId( vector<shared_ptr<PlanarRegion>>& regionList);
};

#endif //SRC_PLANARREGION_H
