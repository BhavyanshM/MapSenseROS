//
// Created by quantum on 12/24/20.
//

#include "ApplicationState.h"

void ApplicationState::update() {
    this->KERNEL_RESOLUTION_LEVEL = this->levels[this->KERNEL_SLIDER_LEVEL - 1];
    this->SUB_H = this->ASPECT_RATIO_HEIGHT * this->KERNEL_RESOLUTION_LEVEL;
    this->SUB_W = this->ASPECT_RATIO_WIDTH * this->KERNEL_RESOLUTION_LEVEL;
    this->PATCH_HEIGHT = (int) this->INPUT_HEIGHT / this->SUB_H;
    this->PATCH_WIDTH = (int) this->INPUT_WIDTH / this->SUB_W;
}

const string &ApplicationState::getDepthFile() const {
    return depthFile;
}

void ApplicationState::setDepthFile(const string &depthFile) {
    ApplicationState::depthFile = depthFile;
}

const string &ApplicationState::getColorFile() const {
    return colorFile;
}

void ApplicationState::setColorFile(const string &colorFile) {
    ApplicationState::colorFile = colorFile;
}
