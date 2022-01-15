#include "GeomTools.h"
#include "PlanarRegion.h"

PlanarRegion::PlanarRegion(int id)
{
   this->normal.setZero();
   this->center.setZero();
   numPatches = 0;
   this->id = id;
}

Vector3f PlanarRegion::GetMeanNormal()
{
   return this->normal / (float) this->numPatches;
}

Vector3f PlanarRegion::GetPCANormal()
{
   Matrix<float, 3, Dynamic> patchMatrix(3, patchCentroids.size());
   for (int i = 0; i < patchCentroids.size(); i++)
   {
      patchMatrix.col(i) = patchCentroids[i];
   }
   Vector3f centroid(patchMatrix.row(0).mean(), patchMatrix.row(1).mean(), patchMatrix.row(2).mean());
   patchMatrix.row(0).array() -= centroid(0);
   patchMatrix.row(1).array() -= centroid(1);
   patchMatrix.row(2).array() -= centroid(2);
   auto svd = patchMatrix.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
   Vector3f plane_normal = svd.matrixU().rightCols<1>();
   return plane_normal;
}

Vector3f PlanarRegion::GetNormal()
{
   if (!normalCalculated)
   {
      normalCalculated = true;
      this->normal = GetPCANormal();
      this->normal.normalize();
      this->normal *= -this->normal.z() / fabs(this->normal.z());
   }
   return this->normal;
}

Vector3f PlanarRegion::GetCenter()
{
   if (!centroidCalculated)
   {
      centroidCalculated = true;
      this->center = getMeanCenter();
   }
   return this->center;
}

Vector3f PlanarRegion::getMeanCenter()
{
   return this->center / (float) this->numPatches;
}

vector<Vector3f> PlanarRegion::getVertices()
{
   return boundaryVertices;
}

vector<Vector2i> PlanarRegion::getLeafPatches()
{
   return leafPatches;
}

int PlanarRegion::GetNumOfBoundaryVertices()
{
   return this->boundaryVertices.size();
}

void PlanarRegion::AddPatch(Vector3f normal, Vector3f center)
{
   this->normal += normal;
   this->center += center;
   this->patchCentroids.emplace_back(center);
   this->numPatches += 1;
   //    printf("Region[%d]:(%d), Center:(%.3lf, %.3lf, %.3lf), Normal:(%.3lf, %.3lf, %.3lf)\n", getId(), getNumPatches(), this->center[0], this->center[1], this->center[2], this->normal[0], this->normal[1], this->normal[2]);
   //    printf("Center:(%.3lf, %.3lf, %.3lf), Normal:(%.3lf, %.3lf, %.3lf)\n", center[0], center[1], center[2], normal[0], normal[1], normal[2]);
}

void PlanarRegion::insertBoundaryVertex(Vector3f vertex)
{
   this->boundaryVertices.push_back(vertex);
}

vector<Vector3f> PlanarRegion::getBoundaryVertices()
{
   return boundaryVertices;
}

void PlanarRegion::insertLeafPatch(Vector2i pos)
{
   this->leafPatches.push_back(pos);
}

int PlanarRegion::getNumPatches()
{
   return numPatches;
}

int PlanarRegion::getId()
{
   return id;
}

void PlanarRegion::setId(int id)
{
   this->id = id;
}

void PlanarRegion::SubSampleBoundary(int skip)
{
   ROS_INFO("Before Boundary Size: %d", boundaryVertices.size());
   for(int i = boundaryVertices.size() - 1; i>= 0; i--)
   {

      if(i % skip == 0 || boundaryVertices[i].norm() > 1000.0f)
      {
         boundaryVertices.erase(boundaryVertices.begin() + i);
      }
   }
   ROS_INFO("After Boundary Size: %d", boundaryVertices.size());
}

void PlanarRegion::SortOrderClockwise()
{
   ROS_INFO("Order Clockwise: %d", boundaryVertices.size());
   Vector3f center = this->getMeanCenter();
   Vector3f normal = this->GetMeanNormal();

   Vector3f first = this->getVertices()[0] - center;
   sort(boundaryVertices.begin(), boundaryVertices.end(), [=](const Vector3f& a, Vector3f& b) -> bool
   {
      return acos(a.dot(first) / (a.norm() * first.norm())) < acos(b.dot(first) / (b.norm() * first.norm()));
   });
   ROS_INFO("Ordered Clockwise\n");
}

void PlanarRegion::GetClockWise2D(vector<Vector2f>& points)
{
   printf("Getting ClockWise 2D\n");
   Vector3f center = this->getMeanCenter();
   Vector3f normal = this->GetMeanNormal();

   for (int i = 0; i < this->GetNumOfBoundaryVertices(); i++)
   {
      Vector3f vec = this->getVertices()[i] - center;
      Vector3f up(0, 0, 1);
      Vector3f dir(0.01f * normal.x(), 0.01f * normal.y(), 0.01f * normal.z());
      Vector3f axis = dir.cross(up).normalized();
      float angle = acos(up.dot(dir) / (up.norm() * dir.norm()));

      if (!isnan(axis.x()) && !isnan(axis.y()) && !isnan(axis.z()))
      {

         AngleAxisf angleAxis(angle, axis);
         Quaternionf quat(angleAxis);
         Vector3f meshVec = quat._transformVector(vec);
         if (meshVec.norm() < 4)
         {
            points.emplace_back(Vector2f(2 * meshVec.x(), 2 * meshVec.y()));
         } else
         {
            printf("Singularity:(%.2lf, %.2lf, %.2lf)\n", meshVec.x(), meshVec.y(), meshVec.norm());
         }
      }
   }
   // printf("Points:%d\n", points.size());
   Vector2f north(1, 0);
   north.normalize();
   sort(points.begin(), points.end(), [=](const Vector2f& a, Vector2f& b) -> bool
   {
      return atan2(a.y(), a.x()) < atan2(b.y(), b.x());
   });
   points.emplace_back(Vector2f(points[0].x(), points[0].y()));
   // for(int i = 0; i<points.size(); i++){
   //     printf("%.2lf, %.2lf\n", points[i].x(), points[i].y());
   // }
   // printf("STOP\n");
   printf("ClockWise 2D Generated\n");
}

void PlanarRegion::SetNormal(const Vector3f& normal)
{
   this->normal = normal;
   normalCalculated = true;
}

void PlanarRegion::SetCenter(const Vector3f& center)
{
   this->center = center;
   centroidCalculated = true;
}

void PlanarRegion::WriteToFile(ofstream& file)
{
   file << "RegionID:" << this->id << endl;
   file << boost::format("Center:%.3f,%.3f,%.3f\n") % center.x() % center.y() % center.z();
   file << boost::format("Normal:%.3f,%.3f,%.3f\n") % normal.x() % normal.y() % normal.z();
   file << "NumPatches:" << this->GetNumOfBoundaryVertices() << endl;
   for (int i = 0; i < boundaryVertices.size(); i++)
   {
      file << boost::format("%.3lf, %.3lf, %.3lf\n") % boundaryVertices[i].x() % boundaryVertices[i].y() % boundaryVertices[i].z();
   }
}

void PlanarRegion::transform(RigidBodyTransform transform)
{
   this->transform(transform.getMatrix().block<3,1>(0,3), transform.getMatrix().block<3,3>(0,0));
}

void PlanarRegion::transform(Vector3d translation, Matrix3d rotation)
{
   this->center = (rotation * this->center.cast<double>() + translation).cast<float>();
   this->normal = (rotation * this->normal.cast<double>()).cast<float>();
   for (int i = 0; i < GetNumOfBoundaryVertices(); i++)
   {
      this->boundaryVertices[i] = (rotation * this->boundaryVertices[i].cast<double>() + translation).cast<float>();
   }
}

void PlanarRegion::CopyAndTransform(shared_ptr<PlanarRegion>& planarRegionToPack, RigidBodyTransform transform)
{
   planarRegionToPack->SetNormal(this->GetNormal());
   planarRegionToPack->SetCenter(this->GetCenter());
   planarRegionToPack->centroidCalculated = true;
   planarRegionToPack->normalCalculated = true;
   planarRegionToPack->SetNumOfMeasurements(this->GetNumOfMeasurements());
   planarRegionToPack->setId(this->getId());
   planarRegionToPack->setPoseId(this->GetPoseId());
   planarRegionToPack->numPatches = this->numPatches;
   for (int i = 0; i < this->boundaryVertices.size(); i++)
   {
      planarRegionToPack->insertBoundaryVertex(this->boundaryVertices[i]);
   }
   planarRegionToPack->transform(transform);
}

void PlanarRegion::ProjectToPlane(Vector4f plane)
{
   this->normal = plane.block<3,1>(0,0);
   this->center = GeomTools::getProjectedPoint(plane, this->GetCenter());
   for(int i = 0; i < GetNumOfBoundaryVertices(); i++)
   {
      this->boundaryVertices[i] = GeomTools::getProjectedPoint(plane, this->boundaryVertices[i]);
   }
}

string PlanarRegion::toString()
{
   boost::format formatter("Id(%d) PoseId(%d) Center(%.3f,%.3f,%.3f) Plane(%.3f,%.3f,%.3f,%.3f) NumPoints(%d) Measured(%d)");
   formatter % this->id % this->GetPoseId() % this->GetCenter().x() % this->GetCenter().y() % this->GetCenter().z() % this->GetNormal().x() % this->GetNormal().y() %
   this->GetNormal().z() % -this->GetNormal().dot(this->GetCenter()) % this->GetNumOfBoundaryVertices() % this->GetNumOfMeasurements();
   return formatter.str();
}

int PlanarRegion::GetPoseId() const
{
   return poseId;
}

void PlanarRegion::setPoseId(int poseId)
{
   PlanarRegion::poseId = poseId;
}

int PlanarRegion::GetNumOfMeasurements() const
{
   return numOfMeasurements;
}

void PlanarRegion::SetNumOfMeasurements(int numOfMeasurements)
{
   PlanarRegion::numOfMeasurements = numOfMeasurements;
}

void PlanarRegion::ComputeBoundaryVerticesPlanar()
{
   float angle = acos(this->GetNormal().dot(Vector3f(0, 0, 1)));
   Vector3f axis = -this->GetNormal().cross(Vector3f(0, 0, 1)).normalized();
   AngleAxisf angleAxis(angle, axis);
   Matrix3d rotation = angleAxis.toRotationMatrix().cast<double>();
   Vector3d translation = Vector3d(this->center.cast<double>());
   transformToWorldFrame.setRotationAndTranslation(rotation, translation);

   for(int i = 0; i<boundaryVertices.size(); i++)
   {
      Vector3f localPoint = transformToWorldFrame.getInverse().transformVector(boundaryVertices[i].cast<double>()).cast<float>();
      this->planarPatchCentroids.emplace_back(Vector2f(localPoint.x(), localPoint.y()));
   }
}

void PlanarRegion::ComputeBoundaryVertices3D(vector<Vector2f> points2D)
{
   vector<Vector3f> points3D;
   for(int i = 0; i<points2D.size(); i++)
   {
      points3D.emplace_back(transformToWorldFrame.transformVector(Vector3d((double)points2D[i].x(), (double)points2D[i].y(), 0)).cast<float>());
   }
   this->boundaryVertices.clear();
   this->boundaryVertices = points3D;
}

void PlanarRegion::RetainConvexHull()
{
   ComputeBoundaryVerticesPlanar();
   vector<Vector2f> convexHull = GeomTools::grahamScanConvexHull(this->planarPatchCentroids);
   ComputeBoundaryVertices3D(convexHull);
}

void PlanarRegion::RetainLinearApproximation()
{
   ComputeBoundaryVerticesPlanar();
   vector<Vector2f> concaveHull = GeomTools::canvasApproximateConcaveHull(this->planarPatchCentroids, 640, 480);
   MatrixXf parametricCurve(2,14);

   GeomTools::getParametricCurve(concaveHull, 13, parametricCurve);
   ComputeBoundaryVertices3D(concaveHull);

   cout << "Parameters:" << endl << parametricCurve << endl;
}

void PlanarRegion::SetToUnitSquare()
{
   SetCenter(Vector3f(0, 0, 0));
   SetNormal(Vector3f(0, 0, 1));
   insertBoundaryVertex(Vector3f(-1,-1,1));
   insertBoundaryVertex(Vector3f(-1,1,1));
   insertBoundaryVertex(Vector3f(1,1,1));
   insertBoundaryVertex(Vector3f(1,-1,1));
}

void PlanarRegion::PrintRegionList(const vector<shared_ptr<PlanarRegion>>& regionList, const std::string& name)
{
   printf("%s: (%d)[", name.c_str(), regionList.size());
   for(auto region : regionList)
   {
      printf("%d,", region->getId());
   }
   printf("]\n");
}

void PlanarRegion::SetZeroId( vector<shared_ptr<PlanarRegion>>& regionList)
{
   for(auto region : regionList)
      region->setId(-1);
}
