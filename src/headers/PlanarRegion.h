#ifndef SRC_PLANARREGION_H
#define SRC_PLANARREGION_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "RegionRing.h"
#include "RigidBodyTransform.h"
#include <memory>
#include <boost/format.hpp>
#include <fstream>

using namespace Eigen;
using namespace std;

class PlanarRegion
{
   private:
      Vector3f normal;
      Vector3f center;
      vector<Vector3f> patchCentroids;
      vector<Vector3f> boundaryVertices;
      vector<Vector2i> leafPatches;
      bool normalCalculated = false;
      bool centroidCalculated = false;
      int numPatches;
      int id;
      int poseId = 0;
      int numOfMeasurements = 1;
   public:
      int getNumOfMeasurements() const;

      void setNumOfMeasurements(int numOfMeasurements);

   public:
      int getPoseId() const;

      void setPoseId(int poseId);

   private:

      Vector3f getPCANormal();

      Vector3f getMeanCenter();

      Vector3f getMeanNormal();

   public:

      PlanarRegion(int id);

      void addPatch(Vector3f normal, Vector3f center);

      void insertBoundaryVertex(Vector3f vertex);

      void insertLeafPatch(Vector2i pos);

      void getClockWise2D(vector<Vector2f>& points);

      int getNumOfBoundaryVertices();

      Vector3f getNormal();

      Vector3f getCenter();

      vector<Vector3f> getVertices();

      vector<Vector2i> getLeafPatches();

      int getNumPatches();

      int getId();

      void setId(int id);

      void setNormal(const Vector3f& normal);

      void setCenter(const Vector3f& center);

      void writeToFile(ofstream& file);

      vector<shared_ptr<RegionRing>> rings;

      void transform(RigidBodyTransform transform);

      void transform(Vector3d translation, Matrix3d rotation);

      void copyAndTransform(shared_ptr<PlanarRegion> planarRegion, RigidBodyTransform transform);

      string toString();

      void projectToPlane(Vector4f plane);
};

#endif //SRC_PLANARREGION_H
