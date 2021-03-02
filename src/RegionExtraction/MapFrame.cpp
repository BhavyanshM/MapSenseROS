#include "MapFrame.h"

void MapFrame::setRegionOutput(Mat& regionOutput)
{
   this->regionOutput = regionOutput;
}

void MapFrame::setPatchData(Mat& patchData)
{
   this->patchData = patchData;
}

Mat& MapFrame::getRegionOutput()
{
   return regionOutput;
}

Mat& MapFrame::getPatchData()
{
   return patchData;
}

void MapFrame::drawGraph(Mat& img, ApplicationState app)
{
   for (int j = 0; j < app.SUB_H - 1; j++)
   {
      for (int i = 0; i < app.SUB_W - 1; i++)
      {
         if (patchData.at<uint8_t>(j, i) == 255)
         {
            line(img, Point(i * app.PATCH_HEIGHT + app.PATCH_HEIGHT / 2, j * app.PATCH_WIDTH + app.PATCH_WIDTH / 2),
                 Point((i + 1) * app.PATCH_HEIGHT + app.PATCH_HEIGHT / 2, j * app.PATCH_WIDTH + app.PATCH_WIDTH / 2), Scalar(0, 150, 0), 1);
            line(img, Point(i * app.PATCH_HEIGHT + app.PATCH_HEIGHT / 2, j * app.PATCH_WIDTH + app.PATCH_WIDTH / 2),
                 Point(i * app.PATCH_HEIGHT + app.PATCH_HEIGHT / 2, (j + 1) * app.PATCH_WIDTH + app.PATCH_WIDTH / 2), Scalar(0, 150, 0), 1);
            circle(img, Point(i * app.PATCH_HEIGHT + app.PATCH_HEIGHT / 2, j * app.PATCH_WIDTH + app.PATCH_WIDTH / 2), 2, Scalar(0, 200, 0), -1);
         }
      }
   }
}

Vec6f MapFrame::getPatch(int x, int y)
{
   return Vec6f(this->getRegionOutput().at<float>(x, y), this->getRegionOutput().at<float>(x, y), this->getRegionOutput().at<float>(x, y),
                this->getRegionOutput().at<float>(x, y), this->getRegionOutput().at<float>(x, y), this->getRegionOutput().at<float>(x, y));
}

