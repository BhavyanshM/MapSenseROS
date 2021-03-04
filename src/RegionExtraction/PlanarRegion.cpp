#include "PlanarRegion.h"

PlanarRegion::PlanarRegion(int id)
{
   this->normal.setZero();
   this->center.setZero();
   numPatches = 0;
   this->id = id;
}

Vector3f PlanarRegion::getMeanNormal()
{
   return this->normal / (float) this->numPatches;
}

Vector3f PlanarRegion::getPCANormal()
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

Vector3f PlanarRegion::getNormal()
{
   if (!normalCalculated)
   {
      normalCalculated = true;
      this->normal = getPCANormal();
      this->normal.normalize();
      this->normal *= -this->normal.z() / fabs(this->normal.z());
   }
   return this->normal;
}

Vector3f PlanarRegion::getCentroid()
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

int PlanarRegion::getNumOfBoundaryVertices()
{
   return this->boundaryVertices.size();
}

void PlanarRegion::addPatch(Vector3f normal, Vector3f center)
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
   return id + 1;
}

void PlanarRegion::setId(int id)
{
   this->id = id;
}

void PlanarRegion::getClockWise2D(vector<Vector2f>& points)
{
   printf("Getting ClockWise 2D\n");
   Vector3f center = this->getMeanCenter();
   Vector3f normal = this->getMeanNormal();

   for (int i = 0; i < this->getNumOfBoundaryVertices(); i++)
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

void PlanarRegion::setNormal(const Vector3f& normal)
{
   this->normal = normal;
   normalCalculated = true;
}

void PlanarRegion::setCenter(const Vector3f& center)
{
   this->center = center;
   centroidCalculated = true;
}

void PlanarRegion::writeToFile(ofstream& file){
    file << "RegionID:" << this->id << endl;
    file << boost::format("Center:%.3f,%.3f,%.3f\n") % center.x() % center.y() % center.z();
    file << boost::format("Normal:%.3f,%.3f,%.3f\n") % normal.x() % normal.y() % normal.z();
    file << "NumPatches:" << this->getNumOfBoundaryVertices() << endl;
    for(int i = 0; i<boundaryVertices.size(); i++){
        file << boost::format("%.3lf, %.3lf, %.3lf\n") % boundaryVertices[i].x() % boundaryVertices[i].y() % boundaryVertices[i].z();
    }
}

void PlanarRegion::transform(Vector3f translation, Vector3f rotationAngles){
   Matrix3f rotation;
   rotation = AngleAxisf(rotationAngles.x(), Vector3f::UnitX())
       * AngleAxisf(rotationAngles.y(), Vector3f::UnitY())
       * AngleAxisf(rotationAngles.z(), Vector3f::UnitZ());
   this->transform(translation, rotation);
}

void PlanarRegion::transform(Vector3f translation, Matrix3f rotation){
   this->center = rotation * this->center;
   this->normal = rotation * this->normal;
   for(int i = 0; i<getNumOfBoundaryVertices(); i++){
      this->boundaryVertices[i] = rotation * this->boundaryVertices[i];
//      this->boundaryVertices[i] += translation;
   }
}

