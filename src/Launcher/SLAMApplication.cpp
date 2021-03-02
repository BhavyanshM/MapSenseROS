//
// Created by quantum on 2/24/21.
//

#include "SLAMApplication.h"

SLAMApplication::SLAMApplication(const Arguments& arguments) : MagnumApplication(arguments)
{
   this->mapper.getFileNames("../../../src/MapSenseROS/Extras/Regions/");
   this->mapper.loadRegions(frameIndex + SKIP_REGIONS, this->mapper.regions);
   this->mapper.loadRegions(frameIndex, this->mapper.latestRegions);
   generateRegionLineMesh(this->mapper.regions, previousRegionEdges, 1);
   generateRegionLineMesh(this->mapper.latestRegions, regionEdges, 2);

   this->mapper.registerRegions(this->mapper.latestRegions);
   cout << this->mapper.latestRegions.size() << endl;
   generateMatchLineMesh(mapper, matchingEdges);
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
}

void SLAMApplication::generateMatchLineMesh(PlanarRegionMapHandler mapper, vector<Object3D*>& edges){
   clear(edges);
   for(int i = 0; i<mapper.matches.size(); i++){
      printf("Reached\n");
      Vector3f first = this->mapper.regions[mapper.matches[i].first]->getCentroid();
      Vector3f second = this->mapper.latestRegions[mapper.matches[i].second]->getCentroid();
      Object3D& matchEdge = _sensor->addChild<Object3D>();
      edges.emplace_back(&matchEdge);
      new RedCubeDrawable{matchEdge, &_drawables, Primitives::line3D({first.x(), first.y(), first.z()}, {second.x(), second.y(), second.z()}),
                          {1.0,1.0,1.0}};
   }
}

void SLAMApplication::generateRegionLineMesh(vector<shared_ptr<PlanarRegion>> planarRegionList, vector<Object3D*>& edges, int color)
{
   clear(edges);
   for (int i = 0; i < planarRegionList.size(); i++)
   {
      shared_ptr<PlanarRegion> planarRegion = planarRegionList[i];
      vector<Vector3f> vertices = planarRegion->getVertices();
      for (int j = SKIP_EDGES; j < vertices.size(); j += SKIP_EDGES)
      {
         Object3D& edge = _sensor->addChild<Object3D>();
         Vector3f prevPoint = vertices[j - SKIP_EDGES];
         Vector3f curPoint = vertices[j];
         edges.emplace_back(&edge);
         new RedCubeDrawable{edge, &_drawables, Primitives::line3D({prevPoint.x(), prevPoint.y(), prevPoint.z()}, {curPoint.x(), curPoint.y(), curPoint.z()}),
                             {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f,
                              (color * 113 % 255) / 255.0f}};
//                             {(planarRegion->getId() * 123 % 255) / 255.0f, (planarRegion->getId() * 161 % 255) / 255.0f,
//                              (planarRegion->getId() * 113 % 255) / 255.0f}};
      }
      Object3D& regionOrigin = _sensor->addChild<Object3D>();
      regionOrigin.translateLocal({planarRegion->getCentroid().x(),planarRegion->getCentroid().y(),planarRegion->getCentroid().z()});
      regionOrigin.scaleLocal({0.002, 0.002, 0.002});
      new RedCubeDrawable{regionOrigin, &_drawables, Primitives::cubeSolid(), {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f,
                                                      (color * 113 % 255) / 255.0f}};
      edges.emplace_back(&regionOrigin);
   }
}

void SLAMApplication::keyPressEvent(KeyEvent& event){
   switch (event.key()){
      case KeyEvent::Key::Right:
         if(frameIndex < this->mapper.files.size() - SKIP_REGIONS){
            frameIndex += SKIP_REGIONS;
         }
         break;
      case KeyEvent::Key::Left:
         if(frameIndex > 1){
            frameIndex -= SKIP_REGIONS;
         }
         break;
      case KeyEvent::Key::O:
//         for(int i = 0; i<regionEdges.size(); i++){
//            regionEdges[i]->transformLocal(Matrix4::rotationX(Rad{5.0_degf}));
//         }
         for(int i = 0; i<this->mapper.latestRegions.size(); i++){
            this->mapper.latestRegions[i]->transform(Vector3f(0,0,0), Vector3f(0.1, 0, 0));
         }
         generateRegionLineMesh(this->mapper.latestRegions, regionEdges, 2);
         break;
   }
   if(event.key() == KeyEvent::Key::Left || event.key() == KeyEvent::Key::Right){
      this->mapper.loadRegions(frameIndex - SKIP_REGIONS, this->mapper.regions);
      this->mapper.loadRegions(frameIndex, this->mapper.latestRegions);
      generateRegionLineMesh(this->mapper.regions, previousRegionEdges, 1);
      generateRegionLineMesh(this->mapper.latestRegions, regionEdges, 2);
      cout << this->mapper.latestRegions.size() << endl;
      this->mapper.registerRegions(this->mapper.latestRegions);
      generateMatchLineMesh(mapper, matchingEdges);
   }

}

int main(int argc, char **argv)
{
   SLAMApplication app({argc, argv});
   return app.exec();
}