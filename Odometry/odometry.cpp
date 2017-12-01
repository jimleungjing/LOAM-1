/********************************************************************/
/*  author:     yusnows                                             */
/*  email:      yusnowsling@gmail.com                               */
/*  oganization:NJU                                                 */
/*  version:    v0.1.0                                              */
/*copyright (c) 2017 yusnows, all right reserved.                   */
/********************************************************************/
#include "odometry.h"
#include <math.h>
#include <iostream>
#include <fstream>

const double TH[4]={0,0.0001,0.03,0.08};

#if 0
int extractfeatruepoints(pcl::PointCloud<PointT>::Ptr cloud,std::vector<int>&edge_index,std::vector<int>&plane_index)
{
    // creates kdtree object
    pcl::KdTreeFLANN<PointT> kdtree;
    // sets our randomly created cloud as the input
    kdtree.setInputCloud (cloud);
    //create a “searchPoint”
    PointT searchPoint;
    // K nearest neighbor search
    int K = 128;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    std::vector<double> cVal;
    for(int cnt=0;cnt<cloud->width*cloud->height;++cnt)
    {
        searchPoint=cloud->points[cnt];
        int delcnt=0;
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            elimiUnormalNearPoint(pointIdxNKNSearch,pointNKNSquaredDistance,cloud);
            cVal.push_back(calculate_cVal(pointIdxNKNSearch,searchPoint,cloud));
        }
    }
}
#endif

int extractfeatruepoints(pcl::PointCloud<PointT>::Ptr cloud,std::vector<int>&edge_index,std::vector<int>&plane_index)
{
    vector<int> Sindex;
    int beg=0;
    vector<double>cVal;
    for(;beg<cloud->width*cloud->height;++beg)
    {
        if(select_S(Sindex,beg,2*MINIBEG,cloud)==0)
            cVal.push_back(calculate_cVal(Sindex,cloud->points[beg],cloud));
        else
            cVal.push_back(-1);
    }
    isEdgeorPlanePoint(cVal,edge_index,plane_index,TH);
    for(int i=0;i<edge_index.size();++i)
        setPoint2Red(cloud->points[edge_index[i]]);
    for(int i=0;i<plane_index.size();++i)
        setPoint2Green(cloud->points[plane_index[i]]);
    std::sort(cVal.begin(),cVal.end());
    for(int i=0;i<cVal.size();++i)
    {
        std::cout<<cVal[i]<<endl;
    }
    return 0;
}

int read2PointCloud(std::string file2read,pcl::PointCloud<PointT>::Ptr cloud)
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
            point.z=tmp_d;
        ++cnt;
        cloud->push_back(point);
    }
    data_in.close();
    return 0;
}


