#include <GeomTools.h>
#include "PlanarRegionMapHandler.h"

void PlanarRegionMapHandler::registerRegions()
{
   Matrix4f T = Matrix4f::Identity();
   int totalNumOfBoundaryPoints = 0;
   for (int i = 0; i < this->matches.size(); i++)
   {
      totalNumOfBoundaryPoints += this->latestRegions[this->matches[i].second]->getNumOfBoundaryVertices();
   }
   MatrixXf A(totalNumOfBoundaryPoints, 6);
   VectorXf b(totalNumOfBoundaryPoints);

   int i = 0;
   for (int m = 0; m < this->matches.size(); m++)
   {
      for (int n = 0; n < this->latestRegions[this->matches[m].second]->getNumOfBoundaryVertices(); n++)
      {
         Vector3f latestPoint = latestRegions[matches[m].second]->getVertices()[n];
         //         printf("(%d,%d,%d):(%.2lf,%.2lf,%.2lf)\n", m,n, i, latestPoint.x(), latestPoint.y(), latestPoint.z());
         Vector3f correspondingMapCentroid = regions[matches[m].first]->getCentroid();
         Vector3f correspondingMapNormal = regions[matches[m].first]->getNormal();
         Vector3f cross = latestPoint.cross(correspondingMapNormal);
         A(i, 0) = cross(0);
         A(i, 1) = cross(1);
         A(i, 2) = cross(2);
         A(i, 3) = correspondingMapNormal(0);
         A(i, 4) = correspondingMapNormal(1);
         A(i, 5) = correspondingMapNormal(2);
         b(i) = -(latestPoint - correspondingMapCentroid).dot(correspondingMapNormal);
         i++;
      }
   }
   VectorXf solution(6);
   solution = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
   eulerAnglesToReference = Vector3f(solution(0), solution(1), solution(2));
   translationToReference = Vector3f(solution(3), solution(4), solution(5));
   cout << solution << endl;
   printf("Translation:(%.2lf, %.2lf, %.2lf)\n", translationToReference(0), translationToReference(1), translationToReference(2));
   printf("EulerAngles:(%.2lf, %.2lf, %.2lf)\n", eulerAnglesToReference(0), eulerAnglesToReference(1), eulerAnglesToReference(2));
}

void PlanarRegionMapHandler::matchPlanarRegionstoMap(vector<shared_ptr<PlanarRegion>> latestRegions)
{
   matches.clear();
   for (int i = 0; i < regions.size(); i++)
   {
      if (regions[i]->getNumOfBoundaryVertices() > 50 && regions[i]->getNumOfBoundaryVertices() < 500)
      {
         for (int j = 0; j < latestRegions.size(); j++)
         {
            if (latestRegions[j]->getNumOfBoundaryVertices() > 50 && latestRegions[j]->getNumOfBoundaryVertices() < 500)
            {
               Vector3f prevNormal = regions[i]->getNormal();
               Vector3f curNormal = latestRegions[j]->getNormal();
               float angularDiff = fabs(prevNormal.dot(curNormal));

               Vector3f prevCenter = regions[i]->getCentroid();
               Vector3f curCenter = latestRegions[j]->getCentroid();
               float dist = (curCenter - prevCenter).norm();
               //         float dist = fabs((prevCenter - curCenter).dot(curNormal)) + fabs((curCenter - prevCenter).dot(prevNormal));

               if (dist < 0.2 && angularDiff > 0.8)
               {
                  matches.emplace_back(i, j);
                  latestRegions[j]->setId(regions[i]->getId());
                  break;
               }
            }
         }
      }
   }
}

void PlanarRegionMapHandler::getFileNames(string dirName)
{
   /* Get the sorted list of file names for planar regions at different frames. */
   this->directory = dirName;
   if (auto dir = opendir(dirName.c_str()))
   {
      while (auto f = readdir(dir))
      {
         cout << f->d_name << endl;
         if (!f->d_name || f->d_name[0] == '.')
            continue;
         files.emplace_back(f->d_name);
      }
      closedir(dir);
   }
   sort(files.begin(), files.end());
}

Vector3f getVec3f(string csv)
{
   vector<string> CSVSubStrings;
   stringstream csvStream(csv);
   string csvStr;
   while (getline(csvStream, csvStr, ','))
   {
      CSVSubStrings.push_back(csvStr);
   }
   //   cout << "Vector:" << Vector3f(stof(CSVSubStrings[0]), stof(CSVSubStrings[1]), stof(CSVSubStrings[2])) << endl;
   return Vector3f(stof(CSVSubStrings[0]), stof(CSVSubStrings[1]), stof(CSVSubStrings[2]));
}

void getNextLineSplit(ifstream& regionFile, vector<string>& subStrings)
{
   subStrings.clear();
   string regionText;
   getline(regionFile, regionText);
   stringstream ss(regionText);
   string str;
   while (getline(ss, str, ':'))
   {
      //      cout << str << '\t';
      subStrings.push_back(str);
   }
   //   cout << endl;
}

void PlanarRegionMapHandler::loadRegions(int frameId, vector<shared_ptr<PlanarRegion>>& regions)
{
   /* Generate planar region objects from the sorted list of files. */
   regions.clear();
   ifstream regionFile(directory + files[frameId]);
   cout << "Loading Regions From: " << directory + files[frameId] << endl;
   vector<string> subStrings;
   getNextLineSplit(regionFile, subStrings); // Get number of regions
   int numRegions = stoi(subStrings[1]);
   for (int r = 0; r < numRegions; r++) // For each region
   {
      shared_ptr<PlanarRegion> region = make_shared<PlanarRegion>(0);
      getNextLineSplit(regionFile, subStrings); // Get regionId
      region->setId(stoi(subStrings[1]));
      getNextLineSplit(regionFile, subStrings); // Get regionCenter
      region->setCenter(getVec3f(subStrings[1]));
      getNextLineSplit(regionFile, subStrings); // Get regionNormal
      region->setNormal(getVec3f(subStrings[1]));
      getNextLineSplit(regionFile, subStrings); // Get numBoundaryVertices
      int length = stoi(subStrings[1]);
      for (int i = 0; i < length; i++)
      {
         getNextLineSplit(regionFile, subStrings);
         region->insertBoundaryVertex(getVec3f(subStrings[0]));
      }
      regions.emplace_back(region);
   }
}

void PlanarRegionMapHandler::transformLatestRegions(Vector3f translation, Vector3f eulerAngles)
{
   for (int i = 0; i < this->latestRegions.size(); i++)
   {
      this->latestRegions[i]->transform(translation, eulerAngles);
   }
}




