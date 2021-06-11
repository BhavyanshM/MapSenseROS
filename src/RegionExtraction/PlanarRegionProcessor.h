//
// Created by quantum on 6/10/21.
//

#ifndef PLANARREGIONPROCESSOR_H
#define PLANARREGIONPROCESSOR_H

#include "PlanarRegion.h"

class PlanarRegionProcessor
{
   private:

      vector<int> planes;

   public:

      void extractRealPlanes(vector<shared_ptr<PlanarRegion>> planarRegionList);

      void filterPlanarRegions(vector<shared_ptr<PlanarRegion>>& planarRegionList);
};

#endif //PLANARREGIONPROCESSOR_H
