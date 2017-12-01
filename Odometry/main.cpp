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

int main ()
{ 
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    std::string dir = "../../data/Pk_select/"; 
    std::string filename = "100_1.XYZ"; 
#if 0
    if (pcl::io::loadPCDFile<PointT> ((dir+filename), *cloud) == -1)
    { 
        // load the file 
        PCL_ERROR ("Couldn't read PCD file \n"); 
        return (-1); 
    }
#else
    if(read2PointCloud((dir+filename),cloud)==-1)
    {
        printf("can not open file %s!\n",(dir+filename).c_str());
        return -1;
    }
#endif
    printf("Loaded %d data points from PCD\n", 
            cloud->width * cloud->height);
    for(int i=0;i<cloud->width*cloud->height;++i)   //set all point to white.
    {
        cloud->points[i].r=255;
        cloud->points[i].g=255;
        cloud->points[i].b=255;
        cloud->points[i].a=128;
    }
    std::vector<double> dis;
    for(int i=1;i<cloud->width*cloud->height;++i)
    {
        dis.push_back(calPoint2PointDis(cloud->points[i-1],cloud->points[i]));
    }
#if 1
    std::sort(dis.begin(),dis.end());
    for(int i=0;i<dis.size();++i)
        cout<<dis[i]<<endl;
    cout<<endl<<endl;
#endif
    std::vector<int>edge_index;
    std::vector<int>plane_index;
    std::cout<<calPoint2PointDis(cloud->points[100],cloud->points[104])<<endl;
    extractfeatruepoints(cloud,edge_index,plane_index);

#if 0
    pcl::PointCloud<PointT>::Ptr cloud_1 (new pcl::PointCloud<PointT>);
    for(int i=0;i<pointIdxNKNSearch.size();++i)
    {
        cloud_1->push_back(cloud->points[pointIdxNKNSearch[i]]);
    }
#endif
//    printf("%d\n",cloud_1->width*cloud_1->height);
    pcl::visualization::PCLVisualizer viewer("Cloud viewer"); 
    viewer.setCameraPosition(0,0,-3.0,0,-1,0);
    viewer.addCoordinateSystem(0.3); 
    viewer.addPointCloud(cloud); 
#if 0
    pcl::visualization::PCLVisualizer viewer_1("Cloud_1 viewer"); 
    viewer_1.setCameraPosition(0,0,-3.0,0,-1,0);
    viewer_1.addCoordinateSystem(0.3); 
    viewer_1.addPointCloud(cloud_1); 
#endif
    while(!viewer.wasStopped())//&&(!viewer_1.wasStopped()))
    {
        viewer.spinOnce(100);
//        viewer_1.spinOnce(100);
    }
    return (0); 
}


