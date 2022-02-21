#include "MeshGenerator.h"

namespace Clay
{
   void MeshGenerator::GenerateRegionLineMesh(Ref<PlanarRegion>& planarRegion, Ref<TriangleMesh>& model)
   {
      CLAY_LOG_INFO("Generating Region Mesh: Vertices: {}", planarRegion->GetNumOfBoundaryVertices());
      for(int i = 0; i< planarRegion->GetNumOfBoundaryVertices(); i++)
      {
         Eigen::Vector3f point = 2 * (planarRegion->getBoundaryVertices()[i]);
         model->InsertVertex(point.x(), point.y(), point.z());
      }
      uint32_t offset = 0;
      for(uint16_t i = 0; i< (planarRegion->GetNumOfBoundaryVertices() - 2) * 3; i+=3)
      {
         model->InsertIndex(0);
         model->InsertIndex(offset + 1);
         model->InsertIndex(offset + 2);
         offset++;
      }
   }

   void MeshGenerator::GenerateMeshForRegions(std::vector<Ref<PlanarRegion>>& planarRegions, Ref<Model> parent)
   {
      for (int i = 0; i< planarRegions.size(); i++)
      {
         Ref<TriangleMesh> regionMesh = std::make_shared<TriangleMesh>(glm::vec4((float)((i+1)*123 % 255) / 255.0f,
                                                                                 (float)((i+1)*326 % 255) / 255.0f,
                                                                                 (float)((i+1)*231 % 255) / 255.0f, 1.0f), parent);
         Ref<PlanarRegion> region = planarRegions[i];

         GenerateRegionLineMesh(region, regionMesh);
         InsertModel(regionMesh);
      }
   }

   void MeshGenerator::InsertModel(Ref<TriangleMesh> model)
   {
      meshes.emplace_back(std::dynamic_pointer_cast<Model>(model));
   }
}


//void MeshGenerator::GenerateRegionLineMesh(vector<Ref<PlanarRegion>> planarRegionList, vector<Object3D *>& edges, int color, Object3D *parent, bool erase)
//{
//   if(erase) clearMesh(edges);
//   for (int i = 0; i < planarRegionList.size(); i+=1)
//   {
//      Ref<PlanarRegion> planarRegion = planarRegionList[i];
//      vector<Vector3f> vertices = planarRegion->getVertices();
//      for (int j = SKIP_EDGES; j < vertices.size() + SKIP_EDGES; j += SKIP_EDGES)
//      {
//         Object3D& edge = parent->addChild<Object3D>();
//         Vector3f prevPoint = vertices[(j - SKIP_EDGES) % vertices.size()];
//         Vector3f curPoint = vertices[j % vertices.size()];
//         edges.emplace_back(&edge);
//         new DrawableObject{edge, drawables,
//                            Magnum::Primitives::line3D({prevPoint.x(), prevPoint.y(), prevPoint.z()}, {curPoint.x(), curPoint.y(), curPoint.z()}),
//                            {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f, (color * 113 % 255) / 255.0f}};
//         //                             {(planarRegion->getId() * 123 % 255) / 255.0f, (planarRegion->getId() * 161 % 255) / 255.0f,
//         //                              (planarRegion->getId() * 113 % 255) / 255.0f}};
//      }
//      Object3D& regionOrigin = parent->addChild<Object3D>();
//      regionOrigin.translateLocal({planarRegion->getCenter().x(), planarRegion->getCenter().y(), planarRegion->GetCenter().z()});
//      regionOrigin.scaleLocal({0.002, 0.002, 0.002});
//      new DrawableObject{regionOrigin, drawables, Magnum::Primitives::cubeSolid(),
//                         {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f, (color * 113 % 255) / 255.0f}};
//      edges.emplace_back(&regionOrigin);
//   }
//}

//void MeshGenerator::clearMesh(vector<Object3D *>& objects)
//{
//   for (int i = 0; i < objects.size(); i++)
//   {
//      delete objects[i];
//   }
//   objects.clear();
//}
//
//void
//MeshGenerator::generateMatchLineMesh(vector<pair<int, int>> matches, vector<Ref<PlanarRegion>> regions, vector<Ref<PlanarRegion>> latestRegions,
//                                     vector<Object3D *>& edges, Object3D *parent)
//{
//   clearMesh(edges);
//   for (int i = 0; i < matches.size(); i++)
//   {
//      Vector3f first = regions[matches[i].first]->GetCenter();
//      Vector3f second = latestRegions[matches[i].second]->GetCenter();
//      Object3D& matchEdge = parent->addChild<Object3D>();
//      edges.emplace_back(&matchEdge);
//      new DrawableObject{matchEdge, drawables, Magnum::Primitives::line3D({first.x(), first.y(), first.z()}, {second.x(), second.y(), second.z()}),
//                          {1.0, 1.0, 1.0}};
//   }
//}
//
//void MeshGenerator::generatePoseMesh(vector<RigidBodyTransform> poses, vector<Object3D *>& objects, Object3D *parent, int color, float scale, float interp)
//{
//   clearMesh(objects);
//   for (int i = 1; i < poses.size(); i++)
//   {
//      Object3D& axes = parent->addChild<Object3D>();
//
//      Vector3d translation = poses[i].getTranslation();
//      axes.translateLocal({static_cast<float>(translation.x()), static_cast<float>(translation.y()), static_cast<float>(translation.z())});
//
//      Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
//      Eigen::Quaterniond quaternion = poses[i].getQuaternion();
//      Eigen::Quaterniond interp_quaternion = identity.slerp(interp, quaternion);
//
//      axes.rotateLocal(Magnum::Quaternion({(float)interp_quaternion.x(), (float)interp_quaternion.y(), (float)interp_quaternion.z()}, interp_quaternion.w()));
//      axes.scaleLocal({(float) 0.1 * scale, (float) 0.1 * scale, (float) 0.1 * scale});
//
//      objects.emplace_back(&axes);
//      new DrawableObject{axes, drawables, Magnum::Primitives::axis3D(), {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f, (color * 113 % 255) / 255.0f}};
//   }
//}
//
//void MeshGenerator::appendPoseMesh(RigidBodyTransform pose, vector<Object3D*>& objects, Object3D *parent, int color)
//{
//   Object3D& axes = parent->addChild<Object3D>();
//
//   Vector3d translation = pose.getMatrix().block<3, 1>(0, 3);
//   axes.translateLocal({static_cast<float>(translation.x()), static_cast<float>(translation.y()), static_cast<float>(translation.z())});
//
//   Eigen::Quaterniond quaternion = pose.getQuaternion().inverse();
//   axes.rotateLocal(Magnum::Quaternion({(float)quaternion.x(), (float)quaternion.y(), (float)quaternion.z()}, -quaternion.w()));
//
//   axes.scaleLocal({0.1, 0.1, 0.1});
//
//   objects.emplace_back(&axes);
//   new DrawableObject{axes, drawables, Magnum::Primitives::axis3D(), {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f, (color * 113 % 255) / 255.0f}};
//}
//
//void MeshGenerator::generatePatchMesh(Object3D* parent, MapFrame& output, vector<Object3D*> objects, const ApplicationState& appState)
//{
//   for (int i = 0; i < output.getRegionOutput().rows; i++)
//   {
//      for (int j = 0; j < output.getRegionOutput().cols; j++)
//      {
//         uint8_t edges = output.getPatchData().at<uint8_t>(i, j);
//         if (edges == 255)
//         {
//            Vec6f patch = output.getRegionOutput().at<Vec6f>(i, j);
//            // cout << patch << endl;
//            Magnum::Vector3 up = {0, 0, 1};
//            Magnum::Vector3 dir = {0.01f * patch[0], 0.01f * patch[1], 0.01f * patch[2]};
//            Magnum::Vector3 axis = Magnum::Math::cross(up, dir).normalized();
//            Magnum::Rad angle = Magnum::Math::acos(Magnum::Math::dot(up, dir) / (up.length() * dir.length()));
//
//            Object3D& plane = parent->addChild<Object3D>();
//            objects.emplace_back(&plane);
//            plane.scale({appState.MAGNUM_PATCH_SCALE, appState.MAGNUM_PATCH_SCALE, appState.MAGNUM_PATCH_SCALE});
//            plane.translate({patch[3], patch[4], patch[5]});
//            // plane.transformLocal(Magnum::Matrix4::rotationX(-Magnum::Rad{180.0_degf}));
//            if (!isnan(axis.x()) && !isnan(axis.y()) && !isnan(axis.z()))
//            {
//               Magnum::Quaternion quat = Magnum::Quaternion::rotation(angle, axis);
//               plane.transformLocal(Magnum::Matrix4(quat.toMatrix()));
//            }
//            new DrawableObject{plane, drawables, Magnum::Primitives::planeSolid(), {0.4, 0.4f, 0.6f}};
//         }
//      }
//   }
//}



