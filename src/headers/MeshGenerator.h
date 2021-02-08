#ifndef SRC_MESHGENERATOR_H
#define SRC_MESHGENERATOR_H

#include <iostream>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"

#include "Magnum/Math/Functions.h"
#include <Magnum/GL/Buffer.h>
#include <Magnum/Trade/MeshData.h>
#include "Magnum/Math/Color.h"
#include "Magnum/Mesh.h"
#include <Magnum/Math/Quaternion.h>

#include <Corrade/Containers/ArrayViewStl.h>

#include "PlanarRegion.h"

using namespace Magnum;
using namespace cv;
using namespace std;

typedef GL::Attribute<0, Vector3> Position;

class MeshGenerator {
public:
    static Trade::MeshData getPlanarRegionMesh(shared_ptr<PlanarRegion> planarRegion);
    static void getPlanarRegionBuffer(shared_ptr<PlanarRegion> planarRegion, GL::Buffer& bufferToPack) ;
};


#endif //SRC_MESHGENERATOR_H
