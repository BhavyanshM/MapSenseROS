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
   EXPECT_NEAR((GeomTools::getProjectedPoint(Vector4f(1, 1, 0, -1), Vector3f(-6, -6, 1)) - Vector3f(0.5f, 0.5f, 1)).norm(),  0, 1e-5f);
}


bool PlanarRegionMapTester::runTests(int argc, char** argv)
{

   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();

}
