//
// Created by isr-lab on 1/10/22.
//

#ifndef MAP_SENSE_NETWORKEDTERRAINSLAMLAYER_H
#define MAP_SENSE_NETWORKEDTERRAINSLAMLAYER_H


#include "ApplicationLayer.h"
#include "PlanarRegionCalculator.h"
#include "MapHandler.h"
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
          MapHandler* _mapper;
         PlanarRegionCalculator *_regionCalculator;
         VisualOdometry* _visualOdometry;
         SLAMModule *_slamModule;
         DataManager* _data;

         Ref<PointCloud> firstCloud;
         std::vector<std::shared_ptr<PlanarRegion>> _regions;

          Ref<PointCloud> _cloud;

          Ref<Texture> _texture;
          cv::Mat _image;

    };


}

#endif //MAP_SENSE_NETWORKEDTERRAINSLAMLAYER_H
