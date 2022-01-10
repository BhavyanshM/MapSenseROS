//
// Created by isr-lab on 1/10/22.
//

#ifndef MAP_SENSE_VISUALTERRAINSLAMLAUNCHER_H
#define MAP_SENSE_VISUALTERRAINSLAMLAUNCHER_H


#include "ApplicationLauncher.h"
#include "PlanarRegionCalculator.h"
#include "VisualOdometry.h"
#include "DataManager.h"
#include "SLAMModule.h"

namespace Clay {
    class VisualTerrainSLAMLauncher : public ApplicationLauncher {

    public:
        void MapsenseInit(int argc, char** argv) override;
        void MapsenseUpdate() override;
        void ImGuiUpdate(ApplicationState& appState) override;

    private:
        PlanarRegionCalculator *_regionCalculator;
        VisualOdometry* _visualOdometry;
        SLAMModule *_slamModule;
        DataManager* _kitti;


    };


}

#endif //MAP_SENSE_VISUALTERRAINSLAMLAUNCHER_H
