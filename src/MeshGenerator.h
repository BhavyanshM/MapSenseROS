#ifndef SRC_MESHGENERATOR_H
#define SRC_MESHGENERATOR_H

#include <Magnum/Trade/MeshData.h>
#include "Magnum/Math/Functions.h"
#include "Magnum/Math/Color.h"
#include "Magnum/Mesh.h"

using namespace Magnum;

class MeshGenerator {
public:
    static Trade::MeshData getPlanarRegionMesh(UnsignedInt segments);
};


#endif //SRC_MESHGENERATOR_H
