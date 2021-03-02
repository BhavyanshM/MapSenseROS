//
// Created by quantum on 12/24/20.
//

#include "ApplicationState.h"

void ApplicationState::update()
{
   if (this->INPUT_HEIGHT > 0 && this->INPUT_WIDTH > 0)
   {
      if ((this->INPUT_HEIGHT % this->KERNEL_SLIDER_LEVEL == 0) && (this->INPUT_WIDTH % this->KERNEL_SLIDER_LEVEL == 0))
      {
         this->PATCH_HEIGHT = this->KERNEL_SLIDER_LEVEL;
         this->PATCH_WIDTH = this->KERNEL_SLIDER_LEVEL;
         this->SUB_H = (int) this->INPUT_HEIGHT / this->PATCH_HEIGHT;
         this->SUB_W = (int) this->INPUT_WIDTH / this->PATCH_WIDTH;
      }
      if ((this->INPUT_HEIGHT % this->FILTER_KERNEL_SIZE == 0) && (this->INPUT_WIDTH % this->FILTER_KERNEL_SIZE == 0))
      {
         this->FILTER_SUB_H = (int) this->INPUT_HEIGHT / this->FILTER_KERNEL_SIZE;
         this->FILTER_SUB_W = (int) this->INPUT_WIDTH / this->FILTER_KERNEL_SIZE;
      }
   }
}

const string& ApplicationState::getDepthFile() const
{
   return depthFile;
}

void ApplicationState::setDepthFile(const string& depthFile)
{
   ApplicationState::depthFile = depthFile;
}

const string& ApplicationState::getColorFile() const
{
   return colorFile;
}

void ApplicationState::setColorFile(const string& colorFile)
{
   ApplicationState::colorFile = colorFile;
}
