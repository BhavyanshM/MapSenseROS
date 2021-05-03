//
// Created by quantum on 3/3/21.
//

#include "../include/GeomTools.h"

Matrix3f GeomTools::getRotationFromAngleApproximations(Vector3f eulerAngles)
{
   float alpha = eulerAngles.x();
   float beta = eulerAngles.y();
   float gamma = eulerAngles.z();
   Matrix3f rotation = Matrix3f::Identity();
   rotation(0, 1) = alpha * beta - gamma;
   rotation(0, 2) = alpha * gamma + beta;
   rotation(1, 0) = gamma;
   rotation(1, 1) += alpha * beta * gamma;
   rotation(1, 2) = beta * gamma - alpha;
   rotation(2, 0) = -beta;
   rotation(2, 1) = alpha;
   return rotation;
}

Vector3f GeomTools::getProjectedPoint(Vector4f plane, Vector3f point)
{
   Vector3f normal = plane.block<3, 1>(0, 0).normalized();
   return point - normal * (normal.dot(point) + plane(3) / plane.block<3, 1>(0, 0).norm());
}

void GeomTools::compressPointSetLinear(shared_ptr<PlanarRegion> region)
{
   printf("Extended Boundary Size: %d\t|\t", region->getNumOfBoundaryVertices());
   vector<Vector3f> boundary = region->getBoundaryVertices();
   region->boundaryVertices.clear();
   uint8_t SKIP = 10;
   for (uint16_t i = 0; i < boundary.size() - SKIP; i++)
   {
      if (((boundary[i] - boundary[i + SKIP / 2]).normalized().dot((boundary[i + SKIP / 2] - boundary[i + SKIP]).normalized())) < 0.5f)
      {
         region->boundaryVertices.emplace_back(boundary[i + 1]);
      }
   }
   printf("Reduced Boundary Size: %d\n", region->getNumOfBoundaryVertices());
}

int nextToTop(stack<int> S)
{
   int top = S.top();
   S.pop();
   int res = S.top();
   S.push(top);
   return res;
}

int orientation(Vector2f p, Vector2f q, Vector2f r)
{
   Vector3f pq;
   Vector3f qr;
   pq << (q - p), 0;
   qr << (r - q), 0;
   float val = pq.cross(qr).z();
   if (val == 0) return 0;  // colinear
   return (val > 0)? 1: 2; // clock or counterclock wise
}

void swap(Vector2f& a, Vector2f& b)
{
   Vector2f temp = a;
   a = b;
   b = temp;
}

void printHull(stack<int> convexHull, vector<Vector2f> points)
{
   while (!convexHull.empty())
   {
      Vector2f p = points[convexHull.top()];
      cout << "(" << p.x() << ", " << p.y() <<")";
      convexHull.pop();
   }
   cout << endl;
}

vector<Vector2f> GeomTools::grahamScanConvexHull(vector<Vector2f> points)
{
   Vector2f minY(10000, 10000);
   int minYIndex = 0;
   for (int i = 0; i<points.size(); i++)
      if (points[i].y() < minY.y())
      {
         minY = points[i];
         minYIndex = i;
      }

   swap(points[0], points[minYIndex]);
   minY = points[0];
   sort(points.begin() + 1, points.end(), [=](Vector2f a, Vector2f b)
   {
      return atan2(a.x() - minY.x(), a.y() - minY.y()) > atan2(b.x() - minY.x(), b.y() - minY.y());
   });

   stack<int> convexHull;
   convexHull.push(0);
   convexHull.push(1);
   convexHull.push(2);

//   for(Vector2f point : points) printf("%.2lf, %.2lf\n", point.x(), point.y());

   vector<int> popels;
   for(int i = 3; i<points.size(); i++)
   {
      while (convexHull.size()>1 && orientation(points[nextToTop(convexHull)], points[convexHull.top()], points[i]) != 1)
      {
         convexHull.pop();
      }
      convexHull.push(i);
   }

   printHull(convexHull, points);
   return points;
}