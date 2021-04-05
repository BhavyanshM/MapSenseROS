#include "MeshGenerator.h"

void MeshGenerator::clearMesh(vector<Object3D *>& objects)
{
   for (int i = 0; i < objects.size(); i++)
   {
      delete objects[i];
   }
   objects.clear();
}

void
MeshGenerator::generateMatchLineMesh(vector<pair<int, int>> matches, vector<shared_ptr<PlanarRegion>> regions, vector<shared_ptr<PlanarRegion>> latestRegions,
                                     vector<Object3D *>& edges, Object3D *parent)
{
   clearMesh(edges);
   for (int i = 0; i < matches.size(); i++)
   {
      Vector3f first = regions[matches[i].first]->getCenter();
      Vector3f second = latestRegions[matches[i].second]->getCenter();
      Object3D& matchEdge = parent->addChild<Object3D>();
      edges.emplace_back(&matchEdge);
      new RedCubeDrawable{matchEdge, drawables, Magnum::Primitives::line3D({first.x(), first.y(), first.z()}, {second.x(), second.y(), second.z()}),
                          {1.0, 1.0, 1.0}};
   }
}

void MeshGenerator::generatePoseMesh(vector<RigidBodyTransform> poses, vector<Object3D *>& objects, Object3D *parent)
{
   clearMesh(objects);
   for (int i = 1; i < poses.size(); i++)
   {
      Vector3d translation = poses[i].getMatrix().block<3, 1>(0, 3);

      Object3D& axes = parent->addChild<Object3D>();
      axes.translateLocal({static_cast<float>(translation.x()), static_cast<float>(translation.y()), static_cast<float>(translation.z())});
      axes.scaleLocal({0.1, 0.1, 0.1});

      objects.emplace_back(&axes);
      new RedCubeDrawable{axes, drawables, Magnum::Primitives::axis3D(), {(i * 123 % 255) / 255.0f, (i * 161 % 255) / 255.0f, (i * 113 % 255) / 255.0f}};
   }
}

void MeshGenerator::generateRegionLineMesh(vector<shared_ptr<PlanarRegion>> planarRegionList, vector<Object3D *>& edges, int color, Object3D *parent)
{
   //   clearMesh(edges);

   for (int i = 0; i < planarRegionList.size(); i++)
   {
      shared_ptr<PlanarRegion> planarRegion = planarRegionList[i];
      vector<Vector3f> vertices = planarRegion->getVertices();
      for (int j = SKIP_EDGES; j < vertices.size(); j += SKIP_EDGES)
      {
         Object3D& edge = parent->addChild<Object3D>();
         Vector3f prevPoint = vertices[j - SKIP_EDGES];
         Vector3f curPoint = vertices[j];
         edges.emplace_back(&edge);
         new RedCubeDrawable{edge, drawables,
                             Magnum::Primitives::line3D({prevPoint.x(), prevPoint.y(), prevPoint.z()}, {curPoint.x(), curPoint.y(), curPoint.z()}),
                             {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f, (color * 113 % 255) / 255.0f}};
         //                             {(planarRegion->getId() * 123 % 255) / 255.0f, (planarRegion->getId() * 161 % 255) / 255.0f,
         //                              (planarRegion->getId() * 113 % 255) / 255.0f}};
      }
      Object3D& regionOrigin = parent->addChild<Object3D>();
      regionOrigin.translateLocal({planarRegion->getCenter().x(), planarRegion->getCenter().y(), planarRegion->getCenter().z()});
      regionOrigin.scaleLocal({0.002, 0.002, 0.002});
      new RedCubeDrawable{regionOrigin, drawables, Magnum::Primitives::cubeSolid(),
                          {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f, (color * 113 % 255) / 255.0f}};
      edges.emplace_back(&regionOrigin);
   }
}

MeshGenerator::MeshGenerator(Magnum::SceneGraph::DrawableGroup3D* drawables) : drawables(drawables)
{
}

