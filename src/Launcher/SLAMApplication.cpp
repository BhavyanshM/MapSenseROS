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

   this->mapper.loadRegions(0, regions);
   this->mapper.loadRegions(4, previousRegions);

//   for (int i = 0; i < regions.size(); i++)
//   {
//      shared_ptr<PlanarRegion> region = regions[i];
//      cout << "ID:" << region->getId() << endl;
//      cout << "NORMAL:" << region->getNormal() << endl;
//      cout << "CENTER:" << region->getCentroid() << endl;
//      cout << "NumBoundary:" << region->getNumOfBoundaryVertices() << endl;
//      for (int j = 0; j < region->getNumOfBoundaryVertices(); j++)
//      {
//         printf("VERTEX:(%d,%d):(%.2lf, %.2lf, %.2lf)\n", i, j, region->getVertices()[j].x(), region->getVertices()[j].y(), region->getVertices()[j].z());
//      }
//   }
}

void SLAMApplication::draw()
{
}

void clear(vector<Object3D *>& objects)
{
   for (int i = 0; i < objects.size(); i++)
   {
      delete objects[i];
   }
   objects.clear();
}

void SLAMApplication::tickEvent()
{
   if(count++ % 10 == 0){
      clear(regionEdges);
      draw_regions(regions, 1);
      draw_regions(previousRegions, 2);
   }
}

void SLAMApplication::draw_regions(vector<shared_ptr<PlanarRegion>> planarRegionList, int color)
{
   for (int i = 0; i < planarRegionList.size(); i++)
   {
      shared_ptr<PlanarRegion> planarRegion = planarRegionList[i];
      vector<Vector3f> vertices = planarRegion->getVertices();
      for (int j = SKIP_EDGES; j < vertices.size(); j += SKIP_EDGES)
      {
         Object3D& edge = _sensor->addChild<Object3D>();
         Vector3f prevPoint = vertices[j - SKIP_EDGES];
         Vector3f curPoint = vertices[j];
         regionEdges.emplace_back(&edge);
         new RedCubeDrawable{edge, &_drawables, Primitives::line3D({prevPoint.x(), prevPoint.y(), prevPoint.z()}, {curPoint.x(), curPoint.y(), curPoint.z()}),
                             {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f,
                              (color * 113 % 255) / 255.0f}};
//                             {(planarRegion->getId() * 123 % 255) / 255.0f, (planarRegion->getId() * 161 % 255) / 255.0f,
//                              (planarRegion->getId() * 113 % 255) / 255.0f}};
      }
   }
}

int main(int argc, char **argv)
{
   SLAMApplication app({argc, argv});
   return app.exec();
}