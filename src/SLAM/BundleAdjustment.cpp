//
// Created by quantum on 1/22/22.
//

#include "BundleAdjustment.h"

BundleAdjustment::BundleAdjustment()
{
   /* Set ISAM2 parameters here. */
   parameters.relinearizeThreshold = 0.01;
   parameters.relinearizeSkip = 1;
   this->isam = gtsam::ISAM2(parameters);

   gtsam::Vector6 odomVariance;
   odomVariance << 1e-2, 1e-2, 1e-2, 1e-1, 1e-1, 1e-1;
   odometryNoise = gtsam::noiseModel::Diagonal::Variances(odomVariance);

   K.reset(new gtsam::Cal3_S2(0,0,0,0,0));
   // Define the camera observation noise model
   cameraMeasurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v
}

void BundleAdjustment::Update(std::vector<PointLandmark>& landmarks, std::vector<Eigen::Matrix4f>& eigenPoses)
{
   /* Conversion to GTSAM types. And call to ComputeNonLinear for BA. */
   using namespace gtsam;
   std::vector<gtsam::Point2> measurements;
   std::vector<gtsam::Pose3> poses;
   std::vector<gtsam::Point3> points;

   for(auto landmark : landmarks)
   {
      measurements.emplace_back(Point2(landmark.GetMeasurements2D()[0].cast<double>()));
      measurements.emplace_back(Point2(landmark.GetMeasurements2D()[1].cast<double>()));
      points.emplace_back(Point3(landmark.GetPoint3D().cast<double>()));
   }
   for(auto pose : eigenPoses)
   {
      poses.emplace_back(Pose3(pose.cast<double>()));
   }

   ComputeNonLinear(measurements, poses, points);
}

void BundleAdjustment::ComputeNonLinear(std::vector<gtsam::Point2>& measurements, std::vector<gtsam::Pose3>& poses, std::vector<gtsam::Point3>& points)
{
   using namespace gtsam;
   using namespace std;
   // Loop over the different poses, adding the observations to iSAM incrementally
   for (size_t i = 0; i < poses.size(); ++i) {

      // Add factors for each landmark observation
      for (size_t j = 0; j < points.size(); ++j) {
         graph.push_back(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measurements[j*2], cameraMeasurementNoise, Symbol('x', i), Symbol('l', j*2), K));
         graph.push_back(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measurements[j*2+1], cameraMeasurementNoise, Symbol('x', i), Symbol('l', j*2+1), K));
      }

      // Add an initial guess for the current pose
      // Intentionally initialize the variables off from the ground truth
      initial.insert(Symbol('x', i), poses[i]);

      // If this is the first iteration, add a prior on the first pose to set the coordinate frame
      // and a prior on the first landmark to set the scale
      // Also, as iSAM solves incrementally, we must wait until each is observed at least twice before
      // adding it to iSAM.
      if( i == 0) {
         // Add a prior on pose x0
         gtsam::Vector6 sigma;
         sigma << 0.3, 0.3, 0.3, 0.1, 0.1, 0.1;

         noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas(sigma); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
         graph.push_back(PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise));

         // Add a prior on landmark l0
         noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
         graph.push_back(PriorFactor<Point3>(Symbol('l', 0), points[0], pointNoise)); // add directly to graph

         // Add initial guesses to all observed landmarks
         // Intentionally initialize the variables off from the ground truth
         for (size_t j = 0; j < points.size(); ++j)
            initial.insert(Symbol('l', j), points[j]);

      } else {
         // Update iSAM with the new factors
         isam.update(graph, initial);
         // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
         // If accuracy is desired at the expense of time, update(*) can be called additional times
         // to perform multiple optimizer iterations every step.
         isam.update();
         Values currentEstimate = isam.calculateEstimate();
         cout << "****************************************************" << endl;
         cout << "Frame " << i << ": " << endl;
         currentEstimate.print("Current estimate: ");

         // Clear the factor graph and values for the next iteration
         graph.resize(0);
         initial.clear();
      }
   }
}

void BundleAdjustment::InsertProjectionFactor()
{


}

void BundleAdjustment::InsertPosePriorFactor()
{
}

void BundleAdjustment::Optimize()
{
}

void BundleAdjustment::Initialize()
{
}
