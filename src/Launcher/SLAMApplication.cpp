//
// Created by quantum on 2/24/21.
//

#include "SLAMApplication.h"

SLAMApplication::SLAMApplication(const Arguments& arguments) : MagnumApplication(arguments)
{
   this->mapper.getFileNames("../../../src/MapSenseROS/Extras/Regions/");
   //   for (int i = 0; i<this->mapper.files.size(); i++){
   //      cout << this->mapper.files[i] << endl;
   //   }
   shared_ptr<PlanarRegion> region = make_shared<PlanarRegion>(0);
   this->mapper.loadRegion(1, region);

   cout << "ID:" << region->getId() << endl;
   cout << "NORMAL:" <<  region->getNormal() << endl;
   cout << "CENTER:" << region->getCentroid() << endl;
   cout << "NumBoundary:" << region->getNumOfBoundaryVertices() << endl;
   for (int i = 0; region->getNumOfBoundaryVertices(); i++){
      cout  << "VERTEX:" << region->getVertices()[i] << endl;
   }

}

void SLAMApplication::draw()
{
}

void SLAMApplication::tickEvent()
{
}

void SLAMApplication::draw_regions(vector<shared_ptr<PlanarRegion>> planarRegionList)
{
   for (int i = 0; i < planarRegionList.size(); i++)
   {
      shared_ptr<PlanarRegion> planarRegion = planarRegionList[i];
      vector<Vector3f> vertices = planarRegion->getVertices();
      for (int j = 1; j < vertices.size(); j += 1)
      {
         Object3D& edge = _sensor->addChild<Object3D>();
         Vector3f prevPoint = vertices[j - 1];
         Vector3f curPoint = vertices[j];
         regionEdges.emplace_back(&edge);
         new RedCubeDrawable{edge, &_drawables, Primitives::line3D({prevPoint.x(), prevPoint.y(), prevPoint.z()}, {curPoint.x(), curPoint.y(), curPoint.z()}),
                             {(planarRegion->getId() * 123 % 255) / 255.0f, (planarRegion->getId() * 161 % 255) / 255.0f,
                              (planarRegion->getId() * 113 % 255) / 255.0f}};
      }
   }
}

int main(int argc, char **argv)
{
   SLAMApplication app({argc, argv});
   return app.exec();
}