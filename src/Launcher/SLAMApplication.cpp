//
// Created by quantum on 2/24/21.
//

#include "SLAMApplication.h"

SLAMApplication::SLAMApplication(const Arguments& arguments) : MagnumApplication(arguments)
{
   this->init(arguments);
   generateRegionLineMesh(this->mapper.regions, previousRegionEdges, 1);
   generateRegionLineMesh(this->mapper.latestRegions, regionEdges, 2);

   this->mapper.matchPlanarRegionstoMap(this->mapper.latestRegions);
   generateMatchLineMesh(mapper, matchingEdges);
}

void SLAMApplication::init(const Arguments& arguments)
{
   string dirName;
   std::vector<std::string> args(arguments.argv, arguments.argv + arguments.argc);
   for (int i = 0; i < arguments.argc; i++)
   {
      if (args[i] == "--regions-dir")
      {
         dirName = args[i + 1];
      }
   }
   // Laptop: "../../../../../src/MapSenseROS/Extras/Regions/" + dirName << endl;
   this->mapper.getFileNames("../../../src/MapSenseROS/Extras/Regions/" + dirName);
   this->mapper.loadRegions(frameIndex, this->mapper.regions);
   this->mapper.loadRegions(frameIndex + SKIP_REGIONS, this->mapper.latestRegions);

   this->fgSLAM.addPriorPoseFactor(Pose3().identity());
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

void SLAMApplication::generateMatchLineMesh(PlanarRegionMapHandler mapper, vector<Object3D *>& edges)
{
   clear(edges);
   for (int i = 0; i < mapper.matches.size(); i++)
   {
      Vector3f first = this->mapper.regions[mapper.matches[i].first]->getCentroid();
      Vector3f second = this->mapper.latestRegions[mapper.matches[i].second]->getCentroid();
      Object3D& matchEdge = _sensor->addChild<Object3D>();
      edges.emplace_back(&matchEdge);
      new RedCubeDrawable{matchEdge, &_drawables, Magnum::Primitives::line3D({first.x(), first.y(), first.z()}, {second.x(), second.y(), second.z()}),
                          {1.0, 1.0, 1.0}};
   }
}

void SLAMApplication::generateRegionLineMesh(vector<shared_ptr<PlanarRegion>> planarRegionList, vector<Object3D *>& edges, int color)
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
         new RedCubeDrawable{edge, &_drawables,
                             Magnum::Primitives::line3D({prevPoint.x(), prevPoint.y(), prevPoint.z()}, {curPoint.x(), curPoint.y(), curPoint.z()}),
                             {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f, (color * 113 % 255) / 255.0f}};
         //                             {(planarRegion->getId() * 123 % 255) / 255.0f, (planarRegion->getId() * 161 % 255) / 255.0f,
         //                              (planarRegion->getId() * 113 % 255) / 255.0f}};
      }
      Object3D& regionOrigin = _sensor->addChild<Object3D>();
      regionOrigin.translateLocal({planarRegion->getCentroid().x(), planarRegion->getCentroid().y(), planarRegion->getCentroid().z()});
      regionOrigin.scaleLocal({0.002, 0.002, 0.002});
      new RedCubeDrawable{regionOrigin, &_drawables, Magnum::Primitives::cubeSolid(),
                          {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f, (color * 113 % 255) / 255.0f}};
      edges.emplace_back(&regionOrigin);
   }

   frameOrigin.scaleLocal({0.002, 0.002, 0.002});
   new RedCubeDrawable{frameOrigin, &_drawables, Magnum::Primitives::cubeSolid(),
                       {(color * 123 % 255) / 255.0f, (color * 161 % 255) / 255.0f, (color * 113 % 255) / 255.0f}};
   edges.emplace_back(&frameOrigin);
}

void SLAMApplication::keyPressEvent(KeyEvent& event)
{
   switch (event.key())
   {
      case KeyEvent::Key::Right:
         if (frameIndex < this->mapper.files.size() - SKIP_REGIONS)
         {
            frameIndex += SKIP_REGIONS;
         }
         break;
      case KeyEvent::Key::Left:
         if (frameIndex > 1)
         {
            frameIndex -= SKIP_REGIONS;
         }
         break;
      case KeyEvent::Key::O:
         //         for(int i = 0; i<regionEdges.size(); i++){
         //            regionEdges[i]->transformLocal(Matrix4::rotationX(Rad{5.0_degf}));
         //         }
         this->mapper.transformLatestRegions(Vector3f(0, 0, 0), Vector3f(0.1, 0, 0));
         generateRegionLineMesh(this->mapper.latestRegions, regionEdges, 2);
         this->mapper.matchPlanarRegionstoMap(this->mapper.latestRegions);
         generateMatchLineMesh(mapper, matchingEdges);
         break;
      case KeyEvent::Key::R:
         auto start = high_resolution_clock::now();

         /* Insert the first set of landmarks to initial pose. */
         updateFactorGraphLandmarks();

         /* Match the previous and latest regions to copy ids and generate match indices. */
         this->mapper.matchPlanarRegionstoMap(this->mapper.latestRegions);

         /* Calculate odometry between previous and latest poses. */
         this->mapper.registerRegions();

         /* Create SE(3) matrix for odometry transform. */
         Matrix3f R;
         R = AngleAxisf(mapper.eulerAnglesToReference.x(), Vector3f::UnitX()) * AngleAxisf(mapper.eulerAnglesToReference.y(), Vector3f::UnitY()) *
             AngleAxisf(mapper.eulerAnglesToReference.z(), Vector3f::UnitZ());
         sensorPoseRelative.setIdentity();
         sensorPoseRelative.block<3, 3>(0, 0) = R.transpose();
         sensorPoseRelative.block<3, 1>(0, 3) = -R.transpose() * this->mapper.translationToReference;

         /* Update total transform from map frame to current sensor pose. */
         mapToSensorTransform *= sensorPoseRelative;

         /* Calculate inverse transform from current sensor pose to map frame. */
         Matrix4f sensorToMapTransform = Matrix4f::Identity();
         sensorToMapTransform = mapToSensorTransform.inverse();

         /* Transform and copy the latest planar regions from current sensor frame to map frame. */
         vector<shared_ptr<PlanarRegion>> transformedRegions;
         this->mapper.transformAndCopyLatestRegions(Vector3f(sensorToMapTransform.block<3, 1>(0, 3)), Matrix3f(sensorToMapTransform.block<3, 3>(0, 0)),
                                                    transformedRegions);

         updateFactorGraphPoses();

         auto end = high_resolution_clock::now();
         auto duration = duration_cast<microseconds>(end - start).count();

         cout << "Registration Took: " << duration / 1000.0f << " ms" << endl;

         generateRegionLineMesh(this->mapper.latestRegions, regionEdges, 2);
         this->mapper.matchPlanarRegionstoMap(this->mapper.latestRegions);
         generateMatchLineMesh(mapper, matchingEdges);
         break;
   }
   if (event.key() == KeyEvent::Key::Space)
   {
   }
   if (event.key() == KeyEvent::Key::Left || event.key() == KeyEvent::Key::Right)
   {
      this->mapper.loadRegions(frameIndex - SKIP_REGIONS, this->mapper.regions);
      this->mapper.loadRegions(frameIndex, this->mapper.latestRegions);
      generateRegionLineMesh(this->mapper.regions, previousRegionEdges, 1);
      generateRegionLineMesh(this->mapper.latestRegions, regionEdges, 2);
      this->mapper.matchPlanarRegionstoMap(this->mapper.latestRegions);
      generateMatchLineMesh(mapper, matchingEdges);
   }
}

void SLAMApplication::updateFactorGraphLandmarks()
{
   for (int i = 0; i < this->mapper.regions.size(); i++)
   {
      shared_ptr<PlanarRegion> region = this->mapper.regions[i];
      Eigen::Vector4d plane;
      plane << Vector3d(region->getNormal()), (double) -region->getNormal().dot(region->getCentroid());
      region->setId(fgSLAM.addOrientedPlaneLandmarkFactor(plane, region->getId()));
   }
}

void SLAMApplication::updateFactorGraphPoses()
{
   fgSLAM.addOdometryFactor(sensorPoseRelative);
}

int main(int argc, char **argv)
{
   std::vector<std::string> args(argv, argv + argc);

   for (int i = 0; i < argc; i++)
   {
      if (args[i] == "--test")
      {
         PlanarRegionMapTester tester;
         if (tester.testICPTransformedPair())
         {
            printf("SUCCESS\n");
         } else
         {
            printf("FAILURE\n");
         }
      } else
      {
         SLAMApplication app({argc, argv});
         return app.exec();
      }
   }
}