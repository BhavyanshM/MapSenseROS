#include "Point3D.h"

Eigen::Vector4d Point3D::getHomogeneous()
{
   Eigen::Vector4d point;
   point << this->data, 1;
   return point;
}

void Point3D::setHomogeneous(Eigen::Vector4d homo)
{
   if(homo(3) != 0)
      this->data = (homo / homo(3)).block<3,1>(0,0);
   else
      this->data = homo.block<3,1>(0,0);
}

double Point3D::distance(Point3D other)
{
   return (this->data - other.data).norm();
}

Eigen::Vector3d Point3D::getData()
{
   return this->data;
}

double Point3D::getX()
{
   return data.x();
}

double Point3D::getY()
{
   return data.y();
}

double Point3D::getZ()
{
   return data.z();
}
