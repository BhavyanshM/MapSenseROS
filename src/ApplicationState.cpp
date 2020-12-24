//
// Created by quantum on 12/24/20.
//

#include "ApplicationState.h"

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
