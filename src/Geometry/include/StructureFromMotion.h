//
// Created by quantum on 4/23/21.
//

#ifndef STRUCTUREFROMMOTION_H
#define STRUCTUREFROMMOTION_H

#include "opencv2/core/core.hpp"
#include <opencv2/imgproc.hpp>
#include "AppUtils.h"

using namespace cv;
using namespace std;

class StructureFromMotion
{
   private:
      vector<Mat> images;
      vector<String> files;

   public:
      StructureFromMotion(String directory);
      void loadImages(String directory);
};

#endif //STRUCTUREFROMMOTION_H
