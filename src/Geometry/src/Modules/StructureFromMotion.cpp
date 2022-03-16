#include "StructureFromMotion.h"

StructureFromMotion::StructureFromMotion(String directory)
{
   loadImages(directory);
   int code = 0;
   int t = 0;
   while(code != 1048689)
   {
      Mat disp;
      hconcat(images[ (t % images.size())], images[ (t + 1) % images.size()], disp);
      namedWindow("Output", WINDOW_NORMAL);
      resizeWindow("Output", disp.cols * 2, disp.rows * 2);
      imshow("Output", disp);
      code = waitKeyEx(0);

      if(code == 1113939) t++;
      if(code == 1113937) t--;
      t %= images.size();

   }
}

void StructureFromMotion::loadImages(String directory)
{
   AppUtils::getFileNames(directory, files);
   for(int i = 0; i<files.size(); i++) {
      images.emplace_back(imread(directory + files[i], IMREAD_ANYCOLOR));
      resize(images[i], images[i], Size(640, 480));
   }
}

