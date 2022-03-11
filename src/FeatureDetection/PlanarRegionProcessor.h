//
// Created by quantum on 6/10/21.
//

#ifndef PLANARREGIONPROCESSOR_H
#define PLANARREGIONPROCESSOR_H

#include "PlanarRegion.h"

class PlanarRegionProcessor
{
   private:

      std::vector<int> planes;

   public:

      void extractRealPlanes(std::vector<std::shared_ptr<PlanarRegion>> planarRegionList);

      void filterPlanarRegions(std::vector<std::shared_ptr<PlanarRegion>>& planarRegionList);
};

#endif //PLANARREGIONPROCESSOR_H
