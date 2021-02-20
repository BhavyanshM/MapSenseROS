//
// Created by quantum on 2/19/21.
//

#include "PlanarRegionSimulator.h"

void PlanarRegionSimulator::createRegions(PlanarRegion& region) {
    region.setNormal(Vector3f(0,0,1));
    region.setCenter(Vector3f(0.2,0.3,-1.4));
    Vector3f phasor(region.getNormal().cross(Vector3f(0,0,1)));
    Matrix3f R;
    R = AngleAxisf(0.25 * M_PI, region.getNormal());
    for(int i = 0; i<10; i++){
        // TODO: Implement circular generator for planar region boundary
    }
}