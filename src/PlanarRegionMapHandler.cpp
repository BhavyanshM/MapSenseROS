//
// Created by quantum on 2/8/21.
//

#include "PlanarRegionMapHandler.h"

void PlanarRegionMapHandler::registerRegions(vector<shared_ptr<PlanarRegion>> latestRegions){
    for(int i = 0; i<regions.size(); i++){
        for(int j = 0; j<latestRegions.size(); j++){
            Vector3f prevCenter = regions[i]->getCentroid();
            Vector3f curCenter = latestRegions[j]->getCentroid();
            Vector3f prevNormal = regions[i]->getNormal();
            Vector3f curNormal = latestRegions[j]->getNormal();
            float perpDist = fabs((prevCenter-curCenter).dot(curNormal)) + fabs((curCenter-prevCenter).dot(prevNormal));
            float angularDiff = fabs(prevNormal.dot(curNormal));
//            ROS_INFO("DIFF:%.4lf", diff);
            if (perpDist < 0.1 && angularDiff > 0.7){
                latestRegions[j]->setId(regions[i]->getId());
                break;
            }
        }
    }
}

void PlanarRegionMapHandler::tester(){
    cout << "Testing Registration and Mapping" << endl;
}

void PlanarRegionMapHandler::loadRegions(){
    for (int i = 0; i<regions.size(); i++){
        shared_ptr<PlanarRegion> region = make_shared<PlanarRegion>(i);
    }
}




