//
// Created by isr-lab on 1/10/22.
//

#include "VisualTerrainSLAMLauncher.h"

namespace Clay {

    void VisualTerrainSLAMLauncher::MapsenseInit(int argc, char** argv)
    {
        _kitti = new DataManager(appState, "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_0/",
                                 "/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_1/",
                                 "/home/quantum/Workspace/Storage/Other/Temp/dataset/data_odometry_poses/poses/00.txt");

        /* KITTI:  float fx = 718.856, fy = 718.856, cx = 607.193, cy = 185.216; */
        _kitti->SetCamera(CameraParams(718.856, 718.856, 607.193, 185.216),
                          CameraParams(718.856, 718.856, 607.193, 185.216));

        _visualOdometry = new VisualOdometry(argc, argv, _networkManager, appState, _kitti);
    }

    void VisualTerrainSLAMLauncher::MapsenseUpdate() {
        //      ROS_DEBUG("TickEvent: %d", count++);
        if (appState.ROS_ENABLED)
        {
            ROS_DEBUG("ROS Update: {}", appState.ROS_ENABLED);
            _networkManager->spin_ros_node();
            _networkManager->acceptMapsenseConfiguration(appState);
            _networkManager->receiverUpdate(appState);
            if (_networkManager->paramsAvailable)
            {
                _networkManager->paramsAvailable = false;
                appState.MERGE_DISTANCE_THRESHOLD = _networkManager->paramsMessage.mergeDistanceThreshold;
                appState.MERGE_ANGULAR_THRESHOLD = _networkManager->paramsMessage.mergeAngularThreshold;
            }

            ROS_DEBUG("ROS Update Completed");
        }
    }

    void VisualTerrainSLAMLauncher::ImGuiUpdate(ApplicationState& appState)
    {
        ImGui::Text("Input:%d,%d Patch:%d,%d Level:%d", appState.INPUT_HEIGHT, appState.INPUT_WIDTH, appState.PATCH_HEIGHT, appState.PATCH_WIDTH,
                    appState.KERNEL_SLIDER_LEVEL);
        ImGui::Checkbox("Generate Regions", &appState.GENERATE_REGIONS);
        ImGui::Checkbox("Filter", &appState.FILTER_SELECTED);
        ImGui::Checkbox("Early Gaussian", &appState.EARLY_GAUSSIAN_BLUR);
        ImGui::SliderInt("Gaussian Size", &appState.GAUSSIAN_SIZE, 1, 4);
        ImGui::SliderInt("Gaussian Sigma", &appState.GAUSSIAN_SIGMA, 1, 20);
        ImGui::SliderFloat("Distance Threshold", &appState.MERGE_DISTANCE_THRESHOLD, 0.0f, 0.1f);
        ImGui::SliderFloat("Angular Threshold", &appState.MERGE_ANGULAR_THRESHOLD, 0.0f, 1.0f);
        ImGui::Checkbox("Components", &appState.SHOW_REGION_COMPONENTS);
        ImGui::Checkbox("Boundary", &appState.SHOW_BOUNDARIES);
        ImGui::Checkbox("Internal", &appState.SHOW_PATCHES);
        if (ImGui::Button("Hide Display"))
        {
            cv::destroyAllWindows();
        }
        ImGui::SliderFloat("Display Window Size", &appState.DISPLAY_WINDOW_SIZE, 0.1, 5.0);
        ImGui::SliderFloat("Depth Brightness", &appState.DEPTH_BRIGHTNESS, 1.0, 100.0);
        ImGui::Checkbox("Visual Debug", &appState.VISUAL_DEBUG);
        ImGui::SliderInt("Visual Debug Delay", &appState.VISUAL_DEBUG_DELAY, 1, 100);
        ImGui::Checkbox("Show Edges", &appState.SHOW_REGION_EDGES);
        ImGui::SliderInt("Skip Edges", &appState.NUM_SKIP_EDGES, 1, 20);
        ImGui::SliderFloat("Magnum Patch Scale", &appState.MAGNUM_PATCH_SCALE, 0.001f, 0.04f);
    }
}

