#ifndef SRC_MESHGENERATOR_H
#define SRC_MESHGENERATOR_H

#include <iostream>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"

#include "Magnum/Math/Functions.h"
#include <Magnum/GL/Buffer.h>
#include <Magnum/Trade/MeshData.h>
#include "Magnum/Math/Color.h"
#include "Magnum/Mesh.h"
#include <Magnum/Math/Quaternion.h>

#include <Corrade/Containers/ArrayViewStl.h>

#include "PlanarRegion.h"
#include "MagnumApplication.h"

using namespace cv;
using namespace std;

typedef Magnum::GL::Attribute<0, Magnum::Vector3> Position;

class MeshGenerator
{
   private:
      const int SKIP_EDGES = 8;

   public:

      void generateRegionLineMesh(vector<shared_ptr<PlanarRegion>> planarRegionList, vector<Object3D *>& regionEdges, int color, Object3D* parent, Magnum::SceneGraph::DrawableGroup3D& drawables);

      void generateMatchLineMesh(PlanarRegionMapHandler mapper, vector<Object3D *>& edges, Object3D* parent, Magnum::SceneGraph::DrawableGroup3D& drawables);

      void generatePoseMesh(vector<RigidBodyTransform> poses, vector<Object3D*>& edges, Object3D* parent, Magnum::SceneGraph::DrawableGroup3D& drawables);

      static Magnum::Trade::MeshData getPlanarRegionMesh(shared_ptr<PlanarRegion> planarRegion);

      static void getPlanarRegionBuffer(shared_ptr<PlanarRegion> planarRegion, Magnum::GL::Buffer& bufferToPack);
};

#endif //SRC_MESHGENERATOR_H
