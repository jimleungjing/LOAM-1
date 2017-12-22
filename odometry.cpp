#include "odometry.h"
#include <iostream>
#include <fstream>
//#include <stdlib.h>
using namespace std;
int main()
{
    pcl::PointCloud<PointT>::Ptr cloud ( new pcl::PointCloud<PointT>);
    std::vector<CloudTypePtr> scanCloud,currScans;
    pcl::PointCloud<PointT>::Ptr sweepCloud1(new pcl::PointCloud<PointT>);


    std::string dir = "../../data/Pk_select/";
    std::string filesuffix[16] = {"_1.XYZ","_3.XYZ","_5.XYZ","_7.XYZ","_9.XYZ","_11.XYZ","_13.XYZ","_15.XYZ", \
                                "_-1.XYZ","_-3.XYZ","_-5.XYZ","_-7.XYZ","_-9.XYZ","_-11.XYZ","_-13.XYZ","_-15.XYZ"};
 
    // read last point cloud : Pk
    int pScan = 0,pPk = 1;std::string filename;
    char ctmp[8];
    scanCloud.reserve(16);
    for(pScan = 0;pScan < 16;pScan++)
    {
        sprintf(ctmp,"%d",pPk);
        std::string stmp(ctmp);
        filename = dir + stmp + filesuffix[pScan];
        pcl::PointCloud<PointT>::Ptr scanCloud0(new pcl::PointCloud<PointT>);
        if((read2PointCloud(filename,scanCloud0) == -1))
        {
            cout<<"Couldn't open file !\n"<<(filename).c_str()<<endl;
            return -1;
        }
        scanCloud.push_back(scanCloud0);
        //std::cout<<"Scan["<<pScan<<"]"<<" size:\n"<<scanCloud[pScan]->width << std::endl;
    }

    std::vector<vector<int> > EdgePointIndices(16);
    std::vector<vector<int> > PlanarPointIndices(16);
    //read current point cloud : Pk+1
    int line = 0;
    pPk = 2;
    currScans.reserve(16);
    for(line = 0;line < 16;line++)
    {
        sprintf(ctmp,"%d",pPk);
        std::string stmp(ctmp);
        filename = dir + stmp + filesuffix[line];
        pcl::PointCloud<PointT>::Ptr scanCloud0(new pcl::PointCloud<PointT>);
        if((read2PointCloud(filename,scanCloud0) == -1))
        {
            cout<<"Couldn't open file !\n"<<(filename).c_str()<<endl;
            return -1;
        }
        currScans.push_back(scanCloud0);
        extractFeatruePoints(scanCloud0,EdgePointIndices[line],PlanarPointIndices[line]);
        *sweepCloud1 += *(currScans[line]);
    }

    FindCorrespondence(0,scanCloud,currScans,NULL,EdgePointIndices,PlanarPointIndices);

    pcl::visualization::PCLVisualizer PkViewer;
    PkViewer.setCameraPosition(0,0,-3.0,0,-1,0);
    PkViewer.addCoordinateSystem(0.3);
    PkViewer.addPointCloud(sweepCloud1);

    while((!PkViewer.wasStopped()))
    {
        PkViewer.spinOnce(100);
    }
    return (0);
}


inline double calculate_cVal(vector<int> &S,const pcl::PointXYZRGB & centerPoint,pcl::PointCloud<PointT>::Ptr cloud)
{
    double cVal;
    int Ssize=S.size();
    double D_centerPoint=sqrt((centerPoint.x)*(centerPoint.x)+(centerPoint.y)*(centerPoint.y)+(centerPoint.z)*(centerPoint.z));
    pcl::PointXYZRGB sumPoint(0,0,0);
    double D_sum=0;
    for(int i=0;i<Ssize;++i)
    {
        sumPoint.x+=(centerPoint.x-cloud->points[S[i]].x);
        sumPoint.y+=(centerPoint.y-cloud->points[S[i]].y);
        sumPoint.z+=(centerPoint.z-cloud->points[S[i]].z);
    }
    D_sum=sqrt((sumPoint.x)*(sumPoint.x)+(sumPoint.y)*(sumPoint.y)+(sumPoint.z)*(sumPoint.z));
    cVal=D_sum/(Ssize*D_centerPoint);
    return cVal;
}



inline int findCIndex(std::vector<double>& cValue, double c)
{
    int size = cValue.size();
    for(int i = 0; i < size; ++i)
    {
      if(cValue[i] == c)
        return i;
    }
    return -1;
}

inline void getPointDistanceSubregion(CloudTypePtr &cloud, vector<int>& S, PointT querypoint,vector<float>& distance)
{
    PointT anotherPoint;
    float dx,dy,dz;
    int size = S.size();
    printf("Qurey(%f,%f,%f):\n",querypoint.x,querypoint.y,querypoint.z);
    for(int i = 0; i < size; ++i)
    {
        anotherPoint = cloud->points[S[i]];
        dx = querypoint.x - anotherPoint.x;
        dy = querypoint.y - anotherPoint.y;
        dz = querypoint.z - anotherPoint.z;
        distance[i] = sqrt(dx*dx + dy*dy + dz*dz);
        printf("%f\t",distance[i]);
    }
}

inline int setPoint2Red(pcl::PointXYZRGB& Point)
{
    Point.r=255;
    Point.g=0;
    Point.b=0;
    return 0;
}
inline int setPoint2Green(pcl::PointXYZRGB& Point)
{
    Point.r=0;
    Point.g=255;
    Point.b=0;
    return 0;
}

inline int setPointColor(PointT& Point,int r,int g, int b)
{
    Point.r = r;
    Point.g = g;
    Point.b = b;
    return 0;
}

int read2PointCloud(std::string& file2read,pcl::PointCloud<PointT>::Ptr cloud)
{
    double tmp_d=0;
    pcl::PointXYZRGB point;
    int cnt=0;
    ifstream data_in(file2read.c_str());
    if(!data_in.is_open())
        return -1;
    while(data_in>>tmp_d)
    {
        if(cnt%3==0)
            point.x=tmp_d;
        else if(cnt%3==1)
            point.y=tmp_d;
        else if(cnt%3==2)
          {
            point.z=tmp_d;
            point.r = point.g = point.b = 255;
            point.a = 128;
            cloud->push_back(point);
          }
        ++cnt;
    }
    data_in.close();
    return 0;
}

int read2PointCloud(std::string file2read,pcl::PointCloud<PointT>& cloud)
{
  double tmp_d=0;
  pcl::PointXYZRGB point;
  int cnt=0;
  ifstream data_in(file2read.c_str());
  if(!data_in.is_open())
      return -1;
  while(data_in>>tmp_d)
  {
      if(cnt%3==0)
          point.x=tmp_d;
      else if(cnt%3==1)
          point.y=tmp_d;
      else if(cnt%3==2)
        {
          point.z=tmp_d;
          point.r = point.g = point.b = 255;
          point.a = 128;
          cloud.push_back(point);
        }
      ++cnt;
  }
  data_in.close();
  return 0;
}

inline int elimiAbnormalPoint(vector<int>& S,vector<float>& distance,pcl::PointCloud<PointT>::Ptr cloud)
{

    for(int i=0;i<distance.size();++i)
    {
        //printf("%2.8f\n",distance[i]);
      if(distance[i]>5)
        {
            S.erase(S.begin()+i);
            distance.erase(distance.begin()+i);
            --i;
        }
    }
    return 0;
}
