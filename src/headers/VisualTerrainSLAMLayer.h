//
// Created by isr-lab on 1/10/22.
//

#ifndef MAP_SENSE_VISUALTERRAINSLAMLAYER_H
#define MAP_SENSE_VISUALTERRAINSLAMLAYER_H


#include "ApplicationLauncher.h"
#include "PlanarRegionCalculator.h"
#include "VisualOdometry.h"
#include "DataManager.h"
#include "SLAMModule.h"


namespace Clay {
    class VisualTerrainSLAMLayer : public ApplicationLauncher {

    public:
        VisualTerrainSLAMLayer(int argc, char **argv);
        void MapsenseUpdate() override;
        void ImGuiUpdate(ApplicationState& appState) override;

    private:
        PlanarRegionCalculator *_regionCalculator;
        VisualOdometry* _visualOdometry;
        SLAMModule *_slamModule;
        DataManager* _data;

        Ref<PointCloud> firstCloud;
        std::vector<std::shared_ptr<PlanarRegion>> _regions;


    };


}

#endif //MAP_SENSE_VISUALTERRAINSLAMLAYER_H
