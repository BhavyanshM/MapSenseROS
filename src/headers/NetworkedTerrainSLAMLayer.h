//
// Created by isr-lab on 1/10/22.
//

#ifndef MAP_SENSE_NETWORKEDTERRAINSLAMLAYER_H
#define MAP_SENSE_NETWORKEDTERRAINSLAMLAYER_H


#include "ApplicationLayer.h"
#include "PlanarRegionCalculator.h"
#include "VisualOdometry.h"
#include "DataManager.h"
#include "SLAMModule.h"

namespace Clay {
    class NetworkedTerrainSLAMLayer : public ApplicationLayer {

    public:
        NetworkedTerrainSLAMLayer(int argc, char **argv);
        void MapsenseUpdate() override;
        void ImGuiUpdate(ApplicationState& appState) override;

    private:
          PlanarRegionMapHandler* _mapper;
         PlanarRegionCalculator *_regionCalculator;
         VisualOdometry* _visualOdometry;
         SLAMModule *_slamModule;
         DataManager* _data;

         Ref<PointCloud> firstCloud;
         std::vector<std::shared_ptr<PlanarRegion>> _regions;

    };


}

#endif //MAP_SENSE_NETWORKEDTERRAINSLAMLAYER_H
