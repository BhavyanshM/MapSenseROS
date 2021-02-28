#include "PlanarRegionMapHandler.h"

void PlanarRegionMapHandler::registerRegions(vector<shared_ptr<PlanarRegion>> latestRegions)
{
   for (int i = 0; i < regions.size(); i++)
   {
      for (int j = 0; j < latestRegions.size(); j++)
      {
         Vector3f prevCenter = regions[i]->getCentroid();
         Vector3f curCenter = latestRegions[j]->getCentroid();
         Vector3f prevNormal = regions[i]->getNormal();
         Vector3f curNormal = latestRegions[j]->getNormal();
         float perpDist = fabs((prevCenter - curCenter).dot(curNormal)) + fabs((curCenter - prevCenter).dot(prevNormal));
         float angularDiff = fabs(prevNormal.dot(curNormal));
         if (perpDist < 0.1 && angularDiff > 0.7)
         {
            latestRegions[j]->setId(regions[i]->getId());
            break;
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
         // cout << f->d_name << endl;
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

void getNextLineSplit(ifstream& regionFile, vector<string>& subStrings){
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
   vector<string> subStrings;
   getNextLineSplit(regionFile, subStrings); // Get number of regions
   int numRegions = stoi(subStrings[1]);
//   cout << "Number of Regions: " << numRegions << endl;
   for (int r = 0; r<numRegions; r++) // For each region
   {
      shared_ptr<PlanarRegion> region = make_shared<PlanarRegion>(0);

      getNextLineSplit(regionFile, subStrings); // Get regionId
//      cout << "RegionID:" << subStrings[1] << endl;
      region->setId(stoi(subStrings[1]));

      getNextLineSplit(regionFile, subStrings); // Get regionCenter
//      cout << subStrings[0] << ":" << subStrings[1] << endl;
      region->setCenter(getVec3f(subStrings[1]));

      getNextLineSplit(regionFile, subStrings); // Get regionNormal
//      cout << subStrings[0] << ":" << subStrings[1] << endl;
      region->setNormal(getVec3f(subStrings[1]));

      getNextLineSplit(regionFile, subStrings); // Get numBoundaryVertices
      int length = stoi(subStrings[1]);
      for (int i = 0; i < length; i++)
      {
         getNextLineSplit(regionFile, subStrings);
//         cout << "RegionPoint(" << r << "," << i << "):" << subStrings[0] << endl;
         region->insertBoundaryVertex(getVec3f(subStrings[0]));
      }
      regions.emplace_back(region);
   }
}




