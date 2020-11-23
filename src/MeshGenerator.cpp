#include "MeshGenerator.h"

Trade::MeshData MeshGenerator::getPlanarRegionMesh(const UnsignedInt segments) {
    CORRADE_ASSERT(segments >= 3, "Primitives::circle3DSolid(): segments must be >= 3",
                   (Trade::MeshData{MeshPrimitive::TriangleFan, 0}));

    /* Calculate attribute count and vertex size */
    std::size_t stride = sizeof(Vector3) + sizeof(Vector3);
    std::size_t attributeCount = 2;
    // if(flags & Circle3DFlag::Tangents) {
    //     stride += sizeof(Vector4);
    //     ++attributeCount;
    // }
    // if(flags & Circle3DFlag::TextureCoordinates) {
    //     stride += sizeof(Vector2);
    //     ++attributeCount;
    // }

    /* Set up the layout */
    Containers::Array<char> vertexData{Containers::NoInit, (segments + 2)*stride};
    Containers::Array<Trade::MeshAttributeData> attributeData{attributeCount};
    std::size_t attributeIndex = 0;
    std::size_t attributeOffset = 0;

    Containers::StridedArrayView1D<Vector3> positions{vertexData,
                                                      reinterpret_cast<Vector3*>(vertexData.data() + attributeOffset),
                                                      segments + 2, std::ptrdiff_t(stride)};
    attributeData[attributeIndex++] = Trade::MeshAttributeData{
            Trade::MeshAttribute::Position, positions};
    attributeOffset += sizeof(Vector3);

    Containers::StridedArrayView1D<Vector3> normals{vertexData,
                                                    reinterpret_cast<Vector3*>(vertexData.data() + attributeOffset),
                                                    segments + 2, std::ptrdiff_t(stride)};
    attributeData[attributeIndex++] = Trade::MeshAttributeData{
            Trade::MeshAttribute::Normal, normals};
    attributeOffset += sizeof(Vector3);

    Containers::StridedArrayView1D<Vector4> tangents;
    // if(flags & Circle3DFlag::Tangents) {
    //     tangents = Containers::StridedArrayView1D<Vector4>{vertexData,
    //                                                        reinterpret_cast<Vector4*>(vertexData.data() + attributeOffset),
    //                                                        segments + 2, std::ptrdiff_t(stride)};
    //     attributeData[attributeIndex++] = Trade::MeshAttributeData{
    //             Trade::MeshAttribute::Tangent, tangents};
    //     attributeOffset += sizeof(Vector4);
    // }

    Containers::StridedArrayView1D<Vector2> textureCoordinates;
    // if(flags & Circle3DFlag::TextureCoordinates) {
    //     textureCoordinates = Containers::StridedArrayView1D<Vector2>{vertexData,
    //                                                                  reinterpret_cast<Vector2*>(vertexData.data() + attributeOffset),
    //                                                                  segments + 2, std::ptrdiff_t(stride)};
    //     attributeData[attributeIndex++] = Trade::MeshAttributeData{
    //             Trade::MeshAttribute::TextureCoordinates, textureCoordinates};
    //     attributeOffset += sizeof(Vector2);
    // }

    CORRADE_INTERNAL_ASSERT(attributeIndex == attributeCount);
    CORRADE_INTERNAL_ASSERT(attributeOffset == stride);

    /* Fill the data. First is center, the first/last point on the edge is
       twice to close the circle properly. */
    positions[0] = {};
    normals[0] = {0.0f, 0.0f, 1.0f};
    // if(flags & Circle3DFlag::Tangents)
    //     tangents[0] = {1.0f, 0.0f, 0.0f, 1.0f};
    // if(flags & Circle3DFlag::TextureCoordinates)
    //     textureCoordinates[0] = {0.5f, 0.5f};
    const Rad angleIncrement(Constants::tau()/segments);
    for(UnsignedInt i = 1; i != segments + 2; ++i) {
        const Rad angle(Float(i - 1)*angleIncrement);
        const std::pair<Float, Float> sincos = Math::sincos(angle);

        positions[i] = {sincos.second, sincos.first, 0.0f};
        normals[i] = {0.0f, 0.0f, 1.0f};
        // if(flags & Circle3DFlag::Tangents)
        //     tangents[i] = {1.0f, 0.0f, 0.0f, 1.0f};
        // if(flags & Circle3DFlag::TextureCoordinates)
        //     textureCoordinates[i] = positions[i].xy()*0.5f + Vector2{0.5f};
    }

    return Trade::MeshData{MeshPrimitive::TriangleFan,
                           std::move(vertexData), std::move(attributeData)};
}