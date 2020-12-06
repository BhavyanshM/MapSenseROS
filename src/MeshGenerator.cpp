#include "MeshGenerator.h"

void MeshGenerator::getPlanarRegionBuffer(shared_ptr<PlanarRegion> planarRegion, GL::Buffer& bufferToPack) {
    vector<Vector3> positions;
    positions.emplace_back(Vector3(planarRegion->getCenter().x(), planarRegion->getCenter().y(), planarRegion->getCenter().z()));

    vector<Vector2f> circularPoints;
    planarRegion->getClockWise2D(circularPoints);

    for(int i = 0; i<circularPoints.size()+2; i++){
        // Vector2f meshPoint = circularPoints[i];
        positions.emplace_back(0.1f, 0.1f, 0.0f);

        // Vector3f center = planarRegion->getCenter();
        // Vector3f normal = planarRegion->getNormal();
        // Vector3f regionPoint =  (vec - center) - ((vec - center).dot(normal)/ (normal.squaredNorm()) * normal);
        // hull.push_back(Point(regionPoint.x(), regionPoint.y()));
    }
    bufferToPack.setData(positions);
}

Trade::MeshData MeshGenerator::getPlanarRegionMesh(shared_ptr<PlanarRegion> planarRegion) {

    vector<Vector2f> circularPoints;
    planarRegion->getClockWise2D(circularPoints);
    // printf("CircularPoints:%d\n", circularPoints.size());

    CORRADE_ASSERT(circularPoints.size() >= 3, "PlanarRegion->getNumOfVertices() must be >= 3",
                   (Trade::MeshData{MeshPrimitive::TriangleFan, 0}));

    /* Calculate attribute count and vertex size */
    std::size_t stride = sizeof(Vector3) + sizeof(Vector3);
    std::size_t attributeCount = 2;

    /* Set up the layout */
    Containers::Array<char> vertexData{Containers::NoInit, (circularPoints.size() + 2)*stride};
    Containers::Array<Trade::MeshAttributeData> attributeData{attributeCount};
    std::size_t attributeIndex = 0;
    std::size_t attributeOffset = 0;

    Containers::StridedArrayView1D<Vector3> positions{vertexData,
                                                      reinterpret_cast<Vector3*>(vertexData.data() + attributeOffset),
                                                      circularPoints.size() + 2, std::ptrdiff_t(stride)};
    attributeData[attributeIndex++] = Trade::MeshAttributeData{
            Trade::MeshAttribute::Position, positions};
    attributeOffset += sizeof(Vector3);

    Containers::StridedArrayView1D<Vector3> normals{vertexData,
                                                    reinterpret_cast<Vector3*>(vertexData.data() + attributeOffset),
                                                    circularPoints.size() + 2, std::ptrdiff_t(stride)};
    attributeData[attributeIndex++] = Trade::MeshAttributeData{
            Trade::MeshAttribute::Normal, normals};
    attributeOffset += sizeof(Vector3);

    CORRADE_INTERNAL_ASSERT(attributeIndex == attributeCount);
    CORRADE_INTERNAL_ASSERT(attributeOffset == stride);

    /* Fill the data. First is center, the first/last point on the edge is
       twice to close the circle properly. */

    // for(UnsignedInt i = 0; i < circularPoints.size() + 1; i++) {
    //     // float meshVecAng = atan2(points[i % points.size()].x(), points[i % points.size()].y());
    //     // float r = 0.00000001f * hullPoint.norm();
    //
    //     // float r = 1;
    //     // const Rad ang(Float(i+1)*angleIncrement);
    //     // const std::pair<Float, Float> sincos = Math::sincos(ang);
    //     // positions[i+1] = {sincos.second, sincos.first, 0.0f};
    //     // normals[i+1] = {0.0f, 0.0f, 1.0f};
    // }


    // printf("Circular:%d\n", circularPoints.size());

    positions[0] = {};
    normals[0] = {0.0f, 0.0f, 1.0f};
    const Rad angleIncrement(Constants::tau()/circularPoints.size());

    for(UnsignedInt i = 1; i != circularPoints.size() + 2; i++) {
        const Rad angle(Float(i - 1)*angleIncrement);
        const std::pair<Float, Float> sincos = Math::sincos(angle);

        positions[i] = {circularPoints[i-1 % circularPoints.size()].x(), circularPoints[i-1 % circularPoints.size()].y(), 0.0f};
        normals[i] = {0.0f, 0.0f, 1.0f};
    }


    return Trade::MeshData{MeshPrimitive::TriangleFan,
                           std::move(vertexData), std::move(attributeData)};
}