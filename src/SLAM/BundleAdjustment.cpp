//
// Created by quantum on 1/22/22.
//

#include "SFMdata.h"
#include "BundleAdjustment.h"

BundleAdjustment::BundleAdjustment(CameraParams& params)
{
//   /* Set ISAM2 parameters here. */
//   parameters.relinearizeThreshold = 0.01;
//   parameters.relinearizeSkip = 1;
//   this->isam = gtsam::ISAM2(parameters);
//
//   gtsam::Vector6 odomVariance;
//   odomVariance << 1e-2, 1e-2, 1e-2, 1e-1, 1e-1, 1e-1;
//   odometryNoise = gtsam::noiseModel::Diagonal::Variances(odomVariance);
//
//   K.reset(new gtsam::Cal3_S2(params._fx,params._fy,0, params._cx, params._cy));
//
//   CLAY_LOG_INFO("CameraParams BA: {}", params._fx);
//
//   // Define the camera observation noise model
//   cameraMeasurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v
}

//void BundleAdjustment::Update(std::vector<PointLandmark>& landmarks, std::vector<Eigen::Matrix4f>& eigenPoses)
//{
//   /* Conversion to GTSAM types. And call to ComputeNonLinear for BA. */
//   std::vector<gtsam::Point2> measurements;
//   std::vector<gtsam::Pose3> poses;
//   std::vector<gtsam::Point3> points;
//
//   for(auto landmark : landmarks)
//   {
//      measurements.emplace_back(gtsam::Point2(landmark.GetMeasurements2D()[0].cast<double>()));
//      measurements.emplace_back(gtsam::Point2(landmark.GetMeasurements2D()[1].cast<double>()));
//      points.emplace_back(gtsam::Point3(landmark.GetPoint3D().cast<double>()));
//   }
//   for(auto pose : eigenPoses)
//   {
//      poses.emplace_back(gtsam::Pose3(pose.cast<double>()));
//   }
//
//   ComputeNonLinear(measurements, poses, points);
//}
//
//void BundleAdjustment::ComputeNonLinear(std::vector<gtsam::Point2>& measurements, std::vector<gtsam::Pose3>& poses, std::vector<gtsam::Point3>& points)
//{
//   using namespace gtsam;
//   using namespace std;
//   // Loop over the different poses, adding the observations to iSAM incrementally
//   for (size_t i = 0; i < poses.size(); ++i) {
//
//      // Add factors for each landmark observation
//      for (size_t j = 0; j < points.size(); ++j) {
//         CLAY_LOG_INFO("Total Points: {} Total Measurements: {} Adding GPFactor {} -> {}", points.size(), measurements.size(), i, j);
//         graph.push_back(GenericProjectionFactor<Pose3, gtsam::Point3, Cal3_S2>(measurements[j*2+i], cameraMeasurementNoise, Symbol('x', i), Symbol('l', j), K));
//      }
//
//      // Add an initial guess for the current pose
//      initial.insert(Symbol('x', i), poses[i]);
//
//      // If this is the first iteration, add a prior on the first pose to set the coordinate frame
//      // and a prior on the first landmark to set the scale
//      // Also, as iSAM solves incrementally, we must wait until each is observed at least twice before
//      // adding it to iSAM.
//      if( i == 0) {
//         // Add a prior on pose x0
//         gtsam::Vector6 sigma;
//         sigma << 0.3, 0.3, 0.3, 0.1, 0.1, 0.1;
//
//         noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas(sigma); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
//         graph.push_back(PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise));
//
//         // Add a prior on landmark l0
//         noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
//         graph.push_back(PriorFactor<gtsam::Point3>(Symbol('l', 0), points[0], pointNoise)); // add directly to graph
//
//         // Add initial guesses to all observed landmarks
//         for (size_t j = 0; j < points.size(); ++j)
//            initial.insert(Symbol('l', j), points[j]);
//
//      } else {
//
//         CLAY_LOG_INFO("Calling Update iSAM2");
//
//         graph.print("Bundle Adjustment Factor Graph");
//         initial.print("Bundle Adjustment Initial Values");
//
////         // Update iSAM with the new factors
////         isam.update(graph, initial);
////         // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
////         // If accuracy is desired at the expense of time, update(*) can be called additional times
////         // to perform multiple optimizer iterations every step.
////         isam.update();
////         Values currentEstimate = isam.calculateEstimate();
//
//         Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
//
//         cout << "****************************************************" << endl;
//         cout << "Frame " << i << ": " << endl;
//         result.print("Current estimate: ");
//
//         // Clear the factor graph and values for the next iteration
//         graph.resize(0);
//         initial.clear();
//      }
//   }
//
//}

void BundleAdjustment::InsertProjectionFactor()
{


}

void BundleAdjustment::InsertPosePriorFactor()
{
}

void BundleAdjustment::Optimize()
{
   using namespace gtsam;
   using namespace std;
   // Define the camera calibration parameters
   Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

   // Define the camera observation noise model, 1 pixel stddev
   auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);

   // Create the set of ground-truth landmarks
   vector<Point3> points = createPoints();

   // Create the set of ground-truth poses
   vector<Pose3> poses = createPoses();

   // Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps
   // to maintain proper linearization and efficient variable ordering, iSAM2
   // performs partial relinearization/reordering at each step. A parameter
   // structure is available that allows the user to set various properties, such
   // as the relinearization threshold and type of linear solver. For this
   // example, we we set the relinearization threshold small so the iSAM2 result
   // will approach the batch result.
   ISAM2Params parameters;
   parameters.relinearizeThreshold = 0.01;
   parameters.relinearizeSkip = 1;
   ISAM2 isam(parameters);

   // Create a Factor Graph and Values to hold the new data
   NonlinearFactorGraph graph;
   Values initialEstimate;

   // Loop over the poses, adding the observations to iSAM incrementally
   for (size_t i = 0; i < poses.size(); ++i) {
      // Add factors for each landmark observation
      for (size_t j = 0; j < points.size(); ++j) {
         PinholeCamera<Cal3_S2> camera(poses[i], *K);
         Point2 measurement = camera.project(points[j]);
         graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(
               measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
      }

      // Add an initial guess for the current pose
      // Intentionally initialize the variables off from the ground truth
      static Pose3 kDeltaPose(Rot3::Rodrigues(-0.1, 0.2, 0.25),
                              Point3(0.05, -0.10, 0.20));
      initialEstimate.insert(Symbol('x', i), poses[i] * kDeltaPose);

      // If this is the first iteration, add a prior on the first pose to set the
      // coordinate frame and a prior on the first landmark to set the scale Also,
      // as iSAM solves incrementally, we must wait until each is observed at
      // least twice before adding it to iSAM.
      if (i == 0) {
         // Add a prior on pose x0, 30cm std on x,y,z and 0.1 rad on roll,pitch,yaw
         static auto kPosePrior = noiseModel::Diagonal::Sigmas(
               (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3))
                     .finished());
         graph.addPrior(Symbol('x', 0), poses[0], kPosePrior);

         // Add a prior on landmark l0
         static auto kPointPrior = noiseModel::Isotropic::Sigma(3, 0.1);
         graph.addPrior(Symbol('l', 0), points[0], kPointPrior);

         // Add initial guesses to all observed landmarks
         // Intentionally initialize the variables off from the ground truth
         static Point3 kDeltaPoint(-0.25, 0.20, 0.15);
         for (size_t j = 0; j < points.size(); ++j)
            initialEstimate.insert<Point3>(Symbol('l', j), points[j] + kDeltaPoint);

      } else {

         graph.print("Graph");
         initialEstimate.print("Initial");
         isam.printStats();
         isam.print("ISAM");

         // Update iSAM with the new factors
         isam.update(graph, initialEstimate);
         // Each call to iSAM2 update(*) performs one iteration of the iterative
         // nonlinear solver. If accuracy is desired at the expense of time,
         // update(*) can be called additional times to perform multiple optimizer
         // iterations every step.
         isam.update();
         Values currentEstimate = isam.calculateEstimate();
         cout << "****************************************************" << endl;
         cout << "Frame " << i << ": " << endl;
         currentEstimate.print("Current estimate: ");

         // Clear the factor graph and values for the next iteration
         graph.resize(0);
         initialEstimate.clear();
      }
   }
}

void BundleAdjustment::Initialize()
{
}
