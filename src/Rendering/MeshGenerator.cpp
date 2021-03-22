#include "MeshGenerator.h"

Trade::MeshData MeshGenerator::getPlanarRegionMesh(shared_ptr<PlanarRegion> planarRegion)
{
   printf("Generating Mesh\n");
   vector<Vector2f> circularPoints;
   planarRegion->getClockWise2D(circularPoints);

   CORRADE_ASSERT(circularPoints.size() >= 3, "PlanarRegion->getNumOfBoundaryVertices() must be >= 3", (Trade::MeshData{MeshPrimitive::TriangleFan, 0}));
   std::size_t stride = sizeof(Vector3) + sizeof(Vector3);
   std::size_t attributeCount = 2;
   Containers::Array<char> vertexData{Containers::NoInit, (circularPoints.size() + 2) * stride};
   Containers::Array<Trade::MeshAttributeData> attributeData{attributeCount};
   std::size_t attributeIndex = 0;
   std::size_t attributeOffset = 0;
   Containers::StridedArrayView1D<Vector3> positions{vertexData, reinterpret_cast<Vector3 *>(vertexData.data() + attributeOffset), circularPoints.size() + 2,
                                                     std::ptrdiff_t(stride)};
   attributeData[attributeIndex++] = Trade::MeshAttributeData{Trade::MeshAttribute::Position, positions};
   attributeOffset += sizeof(Vector3);
   Containers::StridedArrayView1D<Vector3> normals{vertexData, reinterpret_cast<Vector3 *>(vertexData.data() + attributeOffset), circularPoints.size() + 2,
                                                   std::ptrdiff_t(stride)};
   attributeData[attributeIndex++] = Trade::MeshAttributeData{Trade::MeshAttribute::Normal, normals};
   attributeOffset += sizeof(Vector3);

   CORRADE_INTERNAL_ASSERT(attributeIndex == attributeCount);
   CORRADE_INTERNAL_ASSERT(attributeOffset == stride);

   positions[0] = {};
   normals[0] = {0.0f, 0.0f, 1.0f};
   for (UnsignedInt i = 1; i != circularPoints.size() + 2; i++)
   {
      positions[i] = {circularPoints[i - 1 % circularPoints.size()].x(), circularPoints[i - 1 % circularPoints.size()].y(), 0.0f};
      normals[i] = {0.0f, 0.0f, 1.0f};
   }
   printf("Mesh Generated\n");
   return Trade::MeshData{MeshPrimitive::TriangleFan, std::move(vertexData), std::move(attributeData)};
}

void MeshGenerator::getPlanarRegionBuffer(shared_ptr<PlanarRegion> planarRegion, GL::Buffer& bufferToPack)
{
   vector<Vector3> positions;
   positions.emplace_back(Vector3(planarRegion->getCenter().x(), planarRegion->getCenter().y(), planarRegion->getCenter().z()));

   vector<Vector2f> circularPoints;
   planarRegion->getClockWise2D(circularPoints);

   for (int i = 0; i < circularPoints.size() + 2; i++)
   {
      // Vector2f meshPoint = circularPoints[i];
      positions.emplace_back(0.1f, 0.1f, 0.0f);

      // Vector3f center = planarRegion->getCenter();
      // Vector3f normal = planarRegion->getMeanNormal();
      // Vector3f regionPoint =  (vec - center) - ((vec - center).dot(normal)/ (normal.squaredNorm()) * normal);
      // hull.push_back(Point(regionPoint.x(), regionPoint.y()));
   }
   bufferToPack.setData(positions);
}
