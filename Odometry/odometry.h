/********************************************************************/
/*  author:     yusnows                                             */
/*  email:      yusnowsling@gmail.com                               */
/*  oganization:NJU                                                 */
/*  version:    v0.1.0                                              */
/*copyright (c) 2017 yusnows, all right reserved.                   */
/********************************************************************/
#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include <iostream> 
#include <string> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h> 
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include <iostream>
#include <fstream>

#define MINIBEG 32

typedef pcl::PointXYZRGB PointT; 
using namespace std; 

int read2PointCloud(std::string file2read,pcl::PointCloud<PointT>::Ptr cloud);

int extractfeatruepoints(pcl::PointCloud<PointT>::Ptr cloud,\
        std::vector<int>&edge_index,std::vector<int>&plane_index);

inline int select_S(vector<int>&Sindex,int beg,int num);

inline double calPoint2PointDis(PointT& p1,PointT& p2);

inline double calculate_cVal(vector<int>& S,const pcl::PointXYZRGB& centerPoint,\
        pcl::PointCloud<PointT>::Ptr cloud);

inline int elimiUnormalNearPoint(vector<int>& S,vector<float>& distance,\
        pcl::PointCloud<PointT>::Ptr cloud);

inline int isEdgeorPlanePoint(vector<double>&cVal,vector<int>&edge_index,\
        vector<int>&plane_index,const double threadhold[4]);

inline int selectEdgePoint(vector<double>&cVal,vector<int>&edge_index,\
        pcl::PointCloud<PointT>::Ptr clout,pcl::KdTreeFLANN<PointT>& tree);

inline int selectPlanePoint(vector<double>&cVal,vector<int>&plane_index,\
        pcl::PointCloud<PointT>::Ptr clout,pcl::KdTreeFLANN<PointT>& tree);



inline int setPoint2Red(pcl::PointXYZRGB& Point);
inline int setPoint2Green(pcl::PointXYZRGB& Point);


inline int selectEdgePoint(vector<double>&cVal,vector<int>&edge_index,\
        pcl::PointCloud<PointT>::Ptr cloud,pcl::KdTreeFLANN<PointT>& tree)
{

    return 0;
}
inline int selectPlanePoint(vector<double>&cVal,vector<int>&plane_index,\
        pcl::PointCloud<PointT>::Ptr clout,pcl::KdTreeFLANN<PointT>& tree)
{
    return 0;
}

inline int isEdgeorPlanePoint(vector<double>&cVal,vector<int>&edge_index,\
        vector<int>&plane_index,const double threadhold[4])
{

    for(int i=0;i<cVal.size();++i)
    {
        if(cVal[i]>threadhold[2]&&cVal[i]<threadhold[3])
            edge_index.push_back(i);
        else if(cVal[i]>threadhold[0]&&cVal[i]<threadhold[1])
            plane_index.push_back(i);
    }
    return 0;

}


inline int select_S(vector<int>&Sindex,int beg,int num,pcl::PointCloud<PointT>::Ptr cloud)
{
    Sindex.clear();
    if(beg<MINIBEG||beg>(cloud->width*cloud->height-MINIBEG))
        return -1;
    for(int i=beg-num/2;i<beg+num/2;++i)
    {
        if(calPoint2PointDis(cloud->points[beg],cloud->points[i])>0.01)
            continue;
        Sindex.push_back(i);
    }
#if 0
    double dprev,dcurr,r;
    for(int i=1;i<Sindex.size()-1;++i)
    {
        dprev=calPoint2PointDis(cloud->points[i-1],cloud->points[i]);
        dcurr=calPoint2PointDis(cloud->points[i],cloud->points[i+1]);
        if((dcurr/dprev)>8||(dprev/dcurr)>8)
            return -1;//Sindex[i];           //该区间不合适，放弃
    }
#endif
    return 0;
}

inline double calPoint2PointDis(PointT& p1,PointT& p2)
{
    double dis;
    int dx=p1.x-p2.x;
    int dy=p1.y-p2.y;
    int dz=p1.z-p2.z;
    dis=sqrt(dx*dx+dy*dy+dz*dz);
    return dis;
}

inline double calculate_cVal(vector<int>& S,const pcl::PointXYZRGB & centerPoint,\
        pcl::PointCloud<PointT>::Ptr cloud)
{
    double cVal;
    int Ssize=S.size();
    if(Ssize<=16)
    {
        return -1;
    }
    double D_centerPoint=sqrt((centerPoint.x)*(centerPoint.x)+\
            (centerPoint.y)* (centerPoint.y)+(centerPoint.z)*(centerPoint.z));
    pcl::PointXYZRGB sumPoint(0,0,0);
    double D_sum=0;
    for(int i=0;i<Ssize;++i)
    {
        sumPoint.x+=(centerPoint.x-cloud->points[S[i]].x);
        sumPoint.y+=(centerPoint.y-cloud->points[S[i]].y);
        sumPoint.z+=(centerPoint.z-cloud->points[S[i]].z);
    }
    D_sum=sqrt((sumPoint.x)*(sumPoint.x)+(sumPoint.y)*(sumPoint.y)+\
            (sumPoint.z)*(sumPoint.z));
    cVal=D_sum/(Ssize*D_centerPoint);
    return cVal;
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



inline int elimiUnormalNearPoint(vector<int>& S,vector<float>& distance,\
        pcl::PointCloud<PointT>::Ptr cloud)
{
    for(int i=distance.size()-1;i>=0;--i)
    {
        if(distance[i]>0.01)
        {
            S.pop_back();
            distance.pop_back();
        }
    }
    return 0;
}


#endif

