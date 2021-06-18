//
// Created by quantum on 6/10/21.
//

#include "PlanarRegionProcessor.h"

void PlanarRegionProcessor::extractRealPlanes(vector<shared_ptr<PlanarRegion>> planarRegionList)
{
   bool exists = false;
   this->planes.clear();
   Vector4f regionSupportPlane, uniquePlane;
   for (int i = 0; i < planarRegionList.size(); i++)
   {
      shared_ptr<PlanarRegion> region = planarRegionList[i];
      regionSupportPlane << region->getNormal(), -region->getNormal().dot(region->getCenter());
      for (int index : planes)
      {
         shared_ptr<PlanarRegion> plane = planarRegionList[index];
         uniquePlane << plane->getNormal(), -plane->getNormal().dot(plane->getCenter());
         if ((regionSupportPlane - uniquePlane).norm() < 0.1 && (plane->getCenter() - region->getCenter()).norm() < 0.3)
         {
            exists = true;
         }
      }
      if (!exists)
         planes.emplace_back(i);
   }
   printf("Total Unique Surfaces Found: %d/%d\n", planes.size(), planarRegionList.size());
   for(int planeIndex : planes)
   {
      printf("Unique Surface: %s\n", planarRegionList[planeIndex]->toString().c_str());
   }
}

void PlanarRegionProcessor::filterPlanarRegions(vector<shared_ptr<PlanarRegion>>& planarRegionList)
{

}