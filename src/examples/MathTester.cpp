//
// Created by quantum on 6/20/21.
//

#ifndef MATHTESTER_H
#define MATHTESTER_H

#include "Eigen/Dense"

using namespace Eigen;

void testEigenQuaternions();

int main()
{
   testEigenQuaternions();
}

void testEigenQuaternions()
{
   Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
   //   Eigen::Quaterniond quaternion = poses[i].getQuaternion();
   //   Eigen::Quaterniond interp_quaternion = identity.slerp(interp, quaternion);

   Vector3d euler(-0.043,  1.140,  0.011 );

   Quaterniond q_euler;
   q_euler = AngleAxisd(euler.x(), Vector3d::UnitX())
             * AngleAxisd(euler.y(), Vector3d::UnitY())
             * AngleAxisd(euler.z(), Vector3d::UnitZ());

   Quaterniond q2( 0.842 , -0.021,  0.540,  0.016);

   Vector3d eulerAngles = q2.toRotationMatrix().eulerAngles(1,0,2);


   printf("Euler: %.4lf, %.4lf, %.4lf\n", euler.x(), euler.y(), euler.z());
   printf("Quaternion: %.4lf, %.4lf, %.4lf, %.4lf\n", q2.x(), q2.y(), q2.z(), q2.w());
   printf("EulerAngles: %.4lf, %.4lf, %.4lf\n", eulerAngles.x(), eulerAngles.y(), eulerAngles.z());



   //      printf("Interp_Quaternion: %.2lf, %.2lf, %.2lf, %.2lf\n", interp_quaternion.x(), interp_quaternion.y(), interp_quaternion.z(), interp_quaternion.w());

   //      Euler: -0.0430, 1.1400, 0.0110
   //      Quaternion: -0.0151, 0.5396, -0.0070, 0.8418
   //      EulerAngles: 3.0986, 2.0016, -3.1306
}

#endif //MATHTESTER_H
