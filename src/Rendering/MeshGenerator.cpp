#include "MeshGenerator.h"
#include "ClayTools.h"


void MeshGenerator::GeneratePoseMesh(const Eigen::Matrix4f& transform, Clay::Ref<Clay::Model> parent)
{
   glm::mat4 glmTransform = ClayTools::EigenToClay(transform);
   Clay::Ref<Clay::TriangleMesh> pose = std::make_shared<Clay::TriangleMesh>(glm::vec4(0.6f, 0.3f, 0.5f, 1.0f), parent);
   Clay::MeshTools::CoordinateAxes(pose);
   pose->ApplyTransform(glmTransform);
   _poses.push_back(std::move(std::dynamic_pointer_cast<Clay::Model>(pose)));
}

void MeshGenerator::GenerateRegionMesh(std::shared_ptr<PlanarRegion>& planarRegion, Clay::Ref<Clay::TriangleMesh>& model)
{
   //   CLAY_LOG_INFO("Generating Region Mesh: Vertices: {}", planarRegion->GetNumOfBoundaryVertices());
   for (int i = 0; i < planarRegion->GetNumOfBoundaryVertices(); i++)
   {
      Eigen::Vector3f point = (planarRegion->getBoundaryVertices()[i]);
      model->InsertVertex(point.x(), point.y(), point.z());
   }
   uint32_t offset = 0;
   for (uint16_t i = 0; i < (planarRegion->GetNumOfBoundaryVertices() - 2) * 3; i += 3)
   {
      model->InsertIndex(0);
      model->InsertIndex(offset + 1);
      model->InsertIndex(offset + 2);
      offset++;
   }
}

void MeshGenerator::GenerateMeshForRegions(std::vector<Clay::Ref<PlanarRegion>>& planarRegions, Clay::Ref<Clay::Model> parent, bool erase)
{
   if(erase)_meshes.clear();
   for (int i = 0; i < planarRegions.size(); i++)
   {
      Clay::Ref<Clay::TriangleMesh> regionMesh = std::make_shared<Clay::TriangleMesh>(
            glm::vec4((float) ((i + 1) * 123 % 255) / 255.0f, (float) ((i + 1) * 326 % 255) / 255.0f, (float) ((i + 1) * 231 % 255) / 255.0f, 1.0f), parent);
      Clay::Ref<PlanarRegion> region = planarRegions[i];

      GenerateRegionMesh(region, regionMesh);
      InsertModel(regionMesh);
      //      CLAY_LOG_INFO("Mesh Generated Region: {} {}", region->getId(), regionMesh->GetSize());
   }
}

void MeshGenerator::GenerateRegionLineMesh(std::shared_ptr<PlanarRegion>& planarRegion, Clay::Ref<Clay::LineMesh>& model)
{
   for(int i = 0; i<planarRegion->GetNumOfBoundaryVertices() - 1; i++)
   {
      Eigen::Vector3f pointPrev = (planarRegion->getBoundaryVertices()[i]);
      Eigen::Vector3f pointCur = (planarRegion->getBoundaryVertices()[i+1]);
      AppendEdge(model, pointPrev, pointCur);
//      model->InsertVertex(pointPrev.x(), pointPrev.y(), pointPrev.z());
//      model->InsertVertex(pointCur.x(), pointCur.y(), pointCur.z());
   }
   Eigen::Vector3f pointLast = planarRegion->getBoundaryVertices()[planarRegion->GetNumOfBoundaryVertices() - 1];
   Eigen::Vector3f point = planarRegion->getBoundaryVertices()[0];
   AppendEdge(model, pointLast, point);
//   model->InsertVertex(pointLast.x(), pointLast.y(), pointLast.z());
//   model->InsertVertex(point.x(), point.y(), point.z());
}

void MeshGenerator::GenerateLineMeshForRegions(std::vector<Clay::Ref<PlanarRegion>>& planarRegions, Clay::Ref<Clay::Model> parent, bool erase)
{
   if(erase) _lines.clear();
   for (int i = 0; i < planarRegions.size(); i++)
   {
      Clay::Ref<Clay::LineMesh> regionMesh = std::make_shared<Clay::LineMesh>(
            glm::vec4((float) ((i + 1) * 123 % 255) / 255.0f, (float) ((i + 1) * 326 % 255) / 255.0f, (float) ((i + 1) * 231 % 255) / 255.0f, 1.0f), parent);
      Clay::Ref<PlanarRegion> region = planarRegions[i];

      GenerateRegionLineMesh(region, regionMesh);
      InsertLineMesh(regionMesh);
//      CLAY_LOG_INFO("Line Mesh Generated Region: {} {}", region->getId(), regionMesh->GetSize());
   }
}

void MeshGenerator::AppendEdge(Clay::Ref<Clay::LineMesh>& mesh, const Eigen::Vector3f& previous, const Eigen::Vector3f& current)
{
   mesh->InsertVertex(previous.x(), previous.y(), previous.z());
   mesh->InsertVertex(current.x(), current.y(), current.z());

}

void MeshGenerator::GenerateMeshForMatches(const std::vector<Clay::Ref<PlanarRegion>>& current, const std::vector<Clay::Ref<PlanarRegion>>& previous,
                                      const std::vector<std::pair<int, int>>& matches, const Clay::Ref<Clay::Model>& parent)
{
   Clay::Ref<Clay::LineMesh> matchMesh = std::make_shared<Clay::LineMesh>(
         glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), parent);
   for(auto match : matches)
   {
      AppendEdge(matchMesh, previous[match.first]->GetCenter(), current[match.second]->GetCenter());
   }
   InsertLineMesh(matchMesh);
}

void MeshGenerator::InsertModel(Clay::Ref<Clay::TriangleMesh> model)
{
   _meshes.push_back(std::move(std::dynamic_pointer_cast<Clay::Model>(model)));
}

void MeshGenerator::InsertLineMesh(Clay::Ref<Clay::LineMesh> lines)
{
   _lines.push_back(std::move(std::dynamic_pointer_cast<Clay::Model>(lines)));
}

void MeshGenerator::GeneratePatchMesh(cv::Mat& patches)
{
   for (int i = 0; i < patches.rows; i++)
   {
      for (int j = 0; j < patches.cols; j++)
      {
         Eigen::Vector3f normal(patches.at<float>(i, j, 0), patches.at<float>(i, j, 1), patches.at<float>(i, j, 2));
         Eigen::Vector3f centroid(patches.at<float>(i, j, 3), patches.at<float>(i, j, 4), patches.at<float>(i, j, 5));

         Eigen::Vector3f up(0, 0, 1);
         Eigen::Vector3f axis = normal.cross(up).normalized();
         float angle = acos(up.dot(normal) / (up.norm() * normal.norm()));

         Eigen::AngleAxisf angleAxis(angle, axis);
         Eigen::Quaternionf quat(angleAxis);
         Eigen::Vector3f meshVec = quat._transformVector(normal);
      }
   }
}

//void MeshGenerator::generateRegionLineMesh(vector<std::shared_ptr<PlanarRegion>> planarRegionList, vector<Object3D *>& edges, int color, Object3D *parent, bool erase)
//{
//   if(erase) clearMesh(edges);
//   for (int i = 0; i < planarRegionList.size(); i+=1)
//   {
//      std::shared_ptr<PlanarRegion> planarRegion = planarRegionList[i];
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
//MeshGenerator::generateMatchLineMesh(vector<pair<int, int>> matches, vector<std::shared_ptr<PlanarRegion>> regions, vector<std::shared_ptr<PlanarRegion>> latestRegions,
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



