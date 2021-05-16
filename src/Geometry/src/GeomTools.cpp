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
   float normalizedInnerProduct = pq.normalized().dot(qr.normalized());
   if (val == 0)
      return 0;  // colinear
   return (val > 0 || (normalizedInnerProduct > 1)) ? 1 : 2;
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
      cout << "(" << p.x() << ", " << p.y() << ")";
      convexHull.pop();
   }
   cout << endl;
}

vector<Vector2f> getConvexHullPoints(stack<int> indices, vector<Vector2f> points)
{
   vector<Vector2f> convexHull;
   while (!indices.empty())
   {
      convexHull.emplace_back(points[indices.top()]);
      indices.pop();
   }
   return convexHull;
}

vector<Vector2f> GeomTools::grahamScanConvexHull(vector<Vector2f> points)
{
   Vector2f minY(10000, 10000);
   int minYIndex = 0;
   for (int i = 0; i < points.size(); i++)
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

   vector<int> popels;
   for (int i = 3; i < points.size(); i++)
   {
      while (convexHull.size() > 1 && orientation(points[nextToTop(convexHull)], points[convexHull.top()], points[i]) != 1)
      {
         convexHull.pop();
      }
      convexHull.push(i);
   }

   return getConvexHullPoints(convexHull, points);
}

void printCanvas(BoolDynamicMatrix canvas, Vector2i windowPos)
{
   for (int i = 0; i < canvas.rows(); i++)
   {
      for (int j = 0; j < canvas.cols(); j++)
      {
         if (canvas(i, j) == 1)
            printf("o ");
         else if (windowPos.x() != -1 && windowPos.y() != -1 && i > windowPos.x() - 3 && i < windowPos.x() + 3 && j > windowPos.y() - 3 &&
                  j < windowPos.y() + 3)
            printf("X ");
         else
            printf(". ");
      }
      printf("\n");
   }
}

void GeomTools::canvasBoundaryDFS(uint16_t x, uint16_t y, BoolDynamicMatrix& canvas, BoolDynamicMatrix& visited, vector<Vector2f>& concaveHull, AppUtils& appUtils)
{
   if (visited(x, y))
      return;

   visited(x, y) = 1;

   /* Insert this node to data structure here. */
//   printf("DFS(%d,%d)\n", x, y);
   Vector2f candidate(x,y);
   Vector2f current = concaveHull.rbegin()[0];
   Vector2f previous = concaveHull.rbegin()[1];

   concaveHull.emplace_back(candidate);
   printf("(%.2lf, %.2lf) -> (%d, %d) = %.2lf\n", concaveHull.back().x(), concaveHull.back().y(), x, y, ((current - previous).normalized()).dot((candidate - current).normalized()));

   appUtils.displayCanvasWithWindow(canvas, Vector2i(x, y), 3);


//   printCanvas(canvas, Vector2i(x,y));

   /* If debugging, add conditionals here. */

   for (int i = -3; i < 3; i++)
   {
      for (int j = -3; j < 3; j++)
      {
         if (x + i < canvas.rows() - 1 && x + i > 1 && y + j < canvas.cols() - 1 && y + j > 1)
         {
            if (canvas(x + i, y + j) == 1)
            {
               printf("Going Into:(%d,%d)\n", x+i, y+j);
               canvasBoundaryDFS(x + i, y + j, canvas, visited, concaveHull, appUtils);
            }
         }
      }
   }

   printf("Leaving:(%d,%d)\n", x, y);

}

/* Novel algorithm for approximating concave hull by drawing a list of 2D points
 * on a canvas, and traversing the hull with a moving window. */
vector<Vector2f> GeomTools::canvasApproximateConcaveHull(vector<Vector2f> points, uint16_t windowHeight, uint16_t windowWidth)
{
   int r = 120;
   int c = 120;
   BoolDynamicMatrix canvas(r, c);
   canvas.setZero();
   BoolDynamicMatrix visited(r, c);
   visited.setZero();

   AppUtils appUtils;
   appUtils.setDisplayResolution(canvas.rows() * 6, canvas.cols() * 6);

   /* Draw points on canvas using origin and bounding box dimensions. */
   int scale = 45;
   for (int i = 0; i < points.size(); i++)
   {
      canvas((int) (r / 2 + points[i].x() * scale), (int) (c / 2 + points[i].y() * scale)) = 1;
   }

   vector<Vector2f> concaveHull;
   concaveHull.emplace_back(Vector2f(r / 2 + points[0].x() * scale, c / 2 + points[0].y() * scale));

   /* Traverse through the boundary and extract lower vertex-count ordered concave hull. */
   canvasBoundaryDFS((uint16_t) (r / 2 + points[0].x() * scale), (uint16_t) (c / 2 + points[0].y() * scale), canvas, visited, concaveHull, appUtils);

   appUtils.displayPointSet2D(concaveHull);

   return points;
}

