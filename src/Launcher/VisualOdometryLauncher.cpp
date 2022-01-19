#include "stdio.h"
#include "VisualOdometry.h"

void run(VisualOdometry* vo, DataManager* data, NetworkManager* network, ApplicationState& appState);

int main(int argc, char** argv)
{
   printf("VisualOdometry");

   ApplicationState appState;
   appState.STEREO_ODOMETRY_ENABLED = true;
   appState.DATASET_ENABLED = true;

   NetworkManager* nm = new NetworkManager(appState, nullptr);
   DataManager* data = new DataManager(appState, "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_0/",
                           "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_1/",
                           "/home/quantum/Workspace/Storage/Other/Temp/dataset/data_odometry_poses/poses/00.txt");

   /* KITTI:  float fx = 718.856, fy = 718.856, cx = 607.193, cy = 185.216; */
   /* L515 Color: fx = 602.25927734375, cx = 321.3750915527344, fy = 603.0400390625, cy = 240.51527404785156; */
   ROS_INFO("VisualOdometry Created.");
   data->SetCamera(CameraParams(718.856, 718.856, 607.193, 185.216), CameraParams(718.856, 718.856, 607.193, 185.216));
//   data->SetCamera(CameraParams(602.25927734375, 603.0400390625, 321.3750915527344, 240.51527404785156),
//                    CameraParams(602.25927734375, 603.0400390625, 321.3750915527344, 240.51527404785156));

   CLAY_LOG_INFO("Params: {} {} {} {}", data->GetLeftCamera()._fx, data->GetLeftCamera()._cx, data->GetLeftCamera()._fy, data->GetLeftCamera()._cy);

   VisualOdometry* vo = new VisualOdometry(argc, argv, nm, appState, data);

   while(true)
   {
      run(vo, data, nm, appState);
   }

}

void run(VisualOdometry* vo, DataManager* data, NetworkManager* network, ApplicationState& appState)
{
   ROS_DEBUG("Stereo Odom Update");
   bool result = vo->Update(appState, nullptr, nullptr);

   CLAY_LOG_INFO("Result: {}", result);

   vo->Show();

   CLAY_LOG_INFO("Calculated Visual Odometry");
}

