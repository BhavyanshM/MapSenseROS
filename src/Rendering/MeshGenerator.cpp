#include "MeshGenerator.h"
#include <Magnum/Primitives/Plane.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Line.h>
#include <Magnum/Primitives/Circle.h>
#include <Magnum/Primitives/Circle.h>
#include <Magnum/Primitives/Axis.h>

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

void MeshGenerator::generatePoseMesh(vector<RigidBodyTransform> poses, vector<Object3D *>& objects, Object3D *parent, int color, float scale, float interp)
{
   clearMesh(objects);
   for (int i = 1; i < poses.size(); i++)
   {
      Object3D& axes = parent->addChild<Object3D>();

      Vector3d translation = poses[i].getTranslation();
      axes.translateLocal({static_cast<float>(translation.x()), static_cast<float>(translation.y()), static_cast<float>(translation.z())});

      Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
      Eigen::Quaterniond quaternion = poses[i].getQuaternion();
      Eigen::Quaterniond interp_quaternion = identity.slerp(interp, quaternion);

      printf("Quaternion: %.4lf, %.4lf, %.4lf, %.4lf\n", quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

//      Euler: -0.0430, 1.1400, 0.0110
//      Quaternion: -0.0151, 0.5396, -0.0070, 0.8418
//      EulerAngles: 3.0986, 2.0016, -3.1306


      axes.rotateLocal(Magnum::Quaternion({(float)interp_quaternion.x(), (float)interp_quaternion.y(), (float)interp_quaternion.z()}, interp_quaternion.w()));
      axes.scaleLocal({(float) 0.1 * scale, (float) 0.1 * scale, (float) 0.1 * scale});

//      axes.transformLocal(Magnum::Matrix4::rotationX(Magnum::Rad{eulerAngles.x()}));
//      axes.transformLocal(Magnum::Matrix4::rotationY(Magnum::Rad{eulerAngles.y()}));
//      axes.transformLocal(Magnum::Matrix4::rotationZ(Magnum::Rad{eulerAngles.z()}));

      objects.emplace_back(&axes);
      new RedCubeDrawable{axes, drawables, Magnum::Primitives::axis3D(), {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f, (color * 113 % 255) / 255.0f}};
   }
}

void MeshGenerator::appendPoseMesh(RigidBodyTransform pose, vector<Object3D*>& objects, Object3D *parent, int color)
{
   Object3D& axes = parent->addChild<Object3D>();

   Vector3d translation = pose.getMatrix().block<3, 1>(0, 3);
   axes.translateLocal({static_cast<float>(translation.x()), static_cast<float>(translation.y()), static_cast<float>(translation.z())});

   Eigen::Quaterniond quaternion = pose.getQuaternion().inverse();
   axes.rotateLocal(Magnum::Quaternion({(float)quaternion.x(), (float)quaternion.y(), (float)quaternion.z()}, -quaternion.w()));

   axes.scaleLocal({0.1, 0.1, 0.1});

   objects.emplace_back(&axes);
   new RedCubeDrawable{axes, drawables, Magnum::Primitives::axis3D(), {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f, (color * 113 % 255) / 255.0f}};
}

void MeshGenerator::generateRegionLineMesh(vector<shared_ptr<PlanarRegion>> planarRegionList, vector<Object3D *>& edges, int color, Object3D *parent, bool erase)
{
   if(erase) clearMesh(edges);
   for (int i = 0; i < planarRegionList.size(); i+=1)
   {
      shared_ptr<PlanarRegion> planarRegion = planarRegionList[i];
      vector<Vector3f> vertices = planarRegion->getVertices();
      for (int j = SKIP_EDGES; j < vertices.size() + SKIP_EDGES; j += SKIP_EDGES)
      {
         Object3D& edge = parent->addChild<Object3D>();
         Vector3f prevPoint = vertices[(j - SKIP_EDGES) % vertices.size()];
         Vector3f curPoint = vertices[j % vertices.size()];
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

