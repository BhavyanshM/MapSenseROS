#ifndef SRC_MESHGENERATOR_H
#define SRC_MESHGENERATOR_H

#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"

#include "MapsenseHeaders.h"
#include "PlanarRegion.h"

#include "Scene/Mesh/TriangleMesh.h"
#include "Core/Clay.h"

using namespace cv;
using namespace std;

namespace Clay
{
   class MeshGenerator
   {
      public:

         MeshGenerator() {};

         void generateRegionLineMesh(shared_ptr<PlanarRegion>& planarRegion, Ref<TriangleMesh>& model, bool erase = false);

         //      void generateMatchLineMesh(vector<pair<int,int>> matches, vector<shared_ptr<PlanarRegion>> regions, vector<shared_ptr<PlanarRegion>> latestRegions, vector<Object3D *>& edges, Object3D* parent);
         //
         //      void generatePoseMesh(vector<RigidBodyTransform> poses, vector<Object3D*>& edges, Object3D* parent, int color, float scale = 1.0,  float interp = 1.0);
         //
         //      static void clearMesh(vector<Object3D *>& objects);
         //
         //      void appendPoseMesh(RigidBodyTransform pose, vector<Object3D*>& objects, Object3D *parent, int color);

         //      void generatePatchMesh(Object3D* parent, MapFrame& output,  vector<Object3D*> objects, const ApplicationState& appState);


      private:

         const int SKIP_EDGES = 5;

   };

}



#endif //SRC_MESHGENERATOR_H
