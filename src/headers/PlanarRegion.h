#ifndef SRC_PLANARREGION_H
#define SRC_PLANARREGION_H

#include <Eigen/Dense>
#include "RegionRing.h"
#include "RigidBodyTransform.h"
#include "KDTree.h"
#include <memory>

class PlanarRegion
{
   private:
      Eigen::Vector3f normal;
      Eigen::Vector3f center;
      std::vector<Eigen::Vector3f> patchCentroids;
      std::vector<Eigen::Vector2f> planarPatchCentroids;
      std::vector<Eigen::Vector2i> leafPatches;
      std::vector<int> _segmentIndices;

      RigidBodyTransform transformToWorldFrame;
      KDTree tree;
      bool normalCalculated = false;
      bool centroidCalculated = false;
      int numPatches;
      int id;
      int poseId = 0;
      int numOfMeasurements = 1;

   public:
      std::vector<Eigen::Vector3f> boundaryVertices;

      void SubSampleBoundary(int skip);

      void ComputeSegmentIndices(float distThreshold);

      void ComputeBoundaryVerticesPlanar();

      void ComputeBoundaryVertices3D(std::vector<Eigen::Vector2f> points2D);

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

      void GetClockWise2D(std::vector<Eigen::Vector2f>& points);

      void SortOrderClockwise();

      std::vector<Eigen::Vector3f> getBoundaryVertices();

      int GetNumOfBoundaryVertices();

      Eigen::Vector3f GetNormal();

      Eigen::Vector3f GetCenter();

      std::vector<Eigen::Vector3f> getVertices();

      std::vector<Eigen::Vector2i> getLeafPatches();

      int getNumPatches();

      int getId();

      void setId(int id);

      void SetNormal(const Eigen::Vector3f& normal);

      void SetCenter(const Eigen::Vector3f& center);

      void WriteToFile(std::ofstream& file);

      std::vector<std::shared_ptr<RegionRing>> rings;

      void transform(RigidBodyTransform transform);

      void transform(Eigen::Vector3d translation, Eigen::Matrix3d rotation);

      void CopyAndTransform(std::shared_ptr<PlanarRegion>& planarRegionToPack, RigidBodyTransform transform);

      const std::string& toString();

      void ProjectToPlane(Eigen::Vector4f plane);

      void SetToUnitSquare();

      static void PrintRegionList(const std::vector<std::shared_ptr<PlanarRegion>>& regionList, const std::string& name);

      static void SetZeroId( std::vector<std::shared_ptr<PlanarRegion>>& regionList);

      const std::vector<Eigen::Vector2f>& GetPlanarPatchCentroids() const {return planarPatchCentroids;}

      void SetPlanarPatchCentroids(std::vector<Eigen::Vector2f> points) { planarPatchCentroids = points;}

      void SetSegmentIndices(std::vector<int> indices) { _segmentIndices = indices;}

      const std::vector<int>& GetSegmentIndices() const {return _segmentIndices;}

      void CompressRegionSegmentsLinear(float compressDistThreshold, float compressCosineThreshold);
};

#endif //SRC_PLANARREGION_H
