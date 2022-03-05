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

   class MeshGenerator
   {
      public:

         MeshGenerator() {};

         void GenerateMeshForRegions(std::vector<Clay::Ref<PlanarRegion>>& planarRegions, Clay::Ref<Clay::Model> parent);

         void GenerateRegionLineMesh(shared_ptr<PlanarRegion>& planarRegion, Clay::Ref<Clay::TriangleMesh>& model);

         void InsertModel(Clay::Ref<Clay::TriangleMesh> model);

         const std::vector<Clay::Ref<Clay::Model>>& GetModels() const {return meshes;}

         void GeneratePatchMesh(cv::Mat& patches);

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

         std::vector<Clay::Ref<Clay::Model>> meshes;

   };



#endif //SRC_MESHGENERATOR_H
