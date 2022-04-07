#ifndef SRC_MESHGENERATOR_H
#define SRC_MESHGENERATOR_H

#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"

#include "PlanarRegion.h"

#include "Scene/Mesh/MeshTools.h"
#include "Core/Clay.h"

class MeshGenerator
{
   public:

      MeshGenerator()
      {
      };

      void GeneratePoseMesh(const Eigen::Matrix4f& pose, Clay::Ref<Clay::Model> parent);

      void GenerateMeshForRegions(std::vector<Clay::Ref<PlanarRegion>>& planarRegions, Clay::Ref<Clay::Model> parent, bool erase = false);

      void GenerateLineMeshForRegions(std::vector<Clay::Ref<PlanarRegion>>& planarRegions, Clay::Ref<Clay::Model> parent);

      void GenerateRegionMesh(std::shared_ptr<PlanarRegion>& planarRegion, Clay::Ref<Clay::TriangleMesh>& model);

      void GenerateRegionLineMesh(std::shared_ptr<PlanarRegion>& planarRegion, Clay::Ref<Clay::LineMesh>& model);

      void InsertModel(Clay::Ref<Clay::TriangleMesh> model);

      const std::vector<Clay::Ref<Clay::Model>>& GetModels() const
      {
         return _meshes;
      }

      const std::vector<Clay::Ref<Clay::Model>>& GetPoses() const
      {
         return _poses;
      }

      const std::vector<Clay::Ref<Clay::Model>>& GetLines() const
      {
         return _lines;
      }

      void GeneratePatchMesh(cv::Mat& patches);

      void GenerateMeshForMatches(const std::vector<Clay::Ref<PlanarRegion>>& current, const std::vector<Clay::Ref<PlanarRegion>>& previous,
                                  const std::vector<std::pair<int, int>>& matches, const Clay::Ref<Clay::Model>& parent);

      void AppendEdge(Clay::Ref<Clay::LineMesh>& mesh, const Eigen::Vector3f& previous, const Eigen::Vector3f& current);

      void InsertLineMesh(Clay::Ref<Clay::LineMesh> lines);

      //      void generateMatchLineMesh(vector<pair<int,int>> matches, vector<std::shared_ptr<PlanarRegion>> regions, vector<std::shared_ptr<PlanarRegion>> latestRegions, vector<Object3D *>& edges, Object3D* parent);
      //
      //      void generatePoseMesh(vector<RigidBodyTransform> poses, vector<Object3D*>& edges, Object3D* parent, int color, float scale = 1.0,  float interp = 1.0);
      //
      //      static void clearMesh(vector<Object3D *>& objects);
      //
      //      void appendPoseMesh(RigidBodyTransform pose, vector<Object3D*>& objects, Object3D *parent, int color);

      //      void generatePatchMesh(Object3D* parent, MapFrame& output,  vector<Object3D*> objects, const ApplicationState& appState);

   private:

      int SKIP_EDGES = 5;

      std::vector<Clay::Ref<Clay::Model>> _meshes;
      std::vector<Clay::Ref<Clay::Model>> _lines;
      std::vector<Clay::Ref<Clay::Model>> _poses;
};

#endif //SRC_MESHGENERATOR_H
