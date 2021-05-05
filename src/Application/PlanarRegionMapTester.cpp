//
// Created by quantum on 3/3/21.
//

#include "PlanarRegionMapTester.h"

TEST(TestSuite, testCase1)
{
   EXPECT_NEAR((GeomTools::getProjectedPoint(Vector4f(1, 0, 0, -1), Vector3f(0, 1, 1)) - Vector3f(1, 1, 1)).norm(), 0, 1e-5f);
}

TEST(TestSuite, testCase2)
{
   EXPECT_NEAR((GeomTools::getProjectedPoint(Vector4f(1, 1, 0, -1), Vector3f(-6, -6, 1)) - Vector3f(0.5f, 0.5f, 1)).norm(), 0, 1e-5f);
}

TEST(TestSuite, testCase4){
         vector<Vector2f> points;
         points.emplace_back(Vector2f(1,1));
         points.emplace_back(Vector2f(1,-1));
         points.emplace_back(Vector2f(-1,1));
         points.emplace_back(Vector2f(-1,-1));
         points.emplace_back(Vector2f(0,0));
         points.emplace_back(Vector2f(0,-2));
         points.emplace_back(Vector2f(0.5,-0.5));
         points.emplace_back(Vector2f(0.2,0.1));
         points.emplace_back(Vector2f(-1.5,0));

         vector<Vector2f> convexHull = GeomTools::grahamScanConvexHull(points);
}

TEST(TestSuite, testCase3)
{
   KDTree tree(Vector3f(0, 0, 0));
   std::vector<Vector3f> points;

   points.emplace_back( Vector3f(0, 0, 1));
   points.emplace_back( Vector3f(0, 1, 0));
   points.emplace_back( Vector3f(0, 1.2, 1));
   points.emplace_back( Vector3f(2.3, 0, 1.4));
   points.emplace_back( Vector3f(3.1, 1.5, -1.8));
   points.emplace_back( Vector3f(-1.4, 4.3, 0.7));
   points.emplace_back( Vector3f(1.6, 2.3, -0.4));
   points.emplace_back( Vector3f(14.0, -2.1, 1.1));
   points.emplace_back( Vector3f(-1.4, 7.1, -2.7));
   points.emplace_back( Vector3f(1.4, 4.3, 0.7));
   points.emplace_back( Vector3f(1.4, 2.3, 3.7));
   points.emplace_back( Vector3f(1.41, 4.7, -1.3));

   float dist = 1000000000;
   Vector3f queryPoint(3.2, 4.3, 1.2);
   Vector3f bestPoint;
   for(Vector3f point : points)
   {
      tree.root = tree.insert(tree.root, point, 0);
      float dsq = (queryPoint - point).squaredNorm();
      if(dsq < dist){
         dist = dsq;
         bestPoint = point;
      }
   }

   Vector3f treeBestPoint = tree.nearestNeighbor(tree.root, queryPoint, 0)->point;

   cout << "Query Point:" << queryPoint << endl;
   cout << "Best from KDTree:" << treeBestPoint << endl;
   cout << "Best from search:" << bestPoint << endl;


   EXPECT_NEAR((treeBestPoint - bestPoint).norm(), 0, 1e-5f);

}

void PlanarRegionMapTester::testKDTree()
{

}

bool PlanarRegionMapTester::runTests(int argc, char **argv)
{

   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
