#include "odometry.h"
#include <iostream>
#include <fstream>
//#include <stdlib.h>
using namespace std;
bool next_iteration = false;

int main()
{
    pcl::PointCloud<PointT>::Ptr cloud ( new pcl::PointCloud<PointT>);
    std::vector<CloudTypePtr> lastScans,currScans,reprojectedScans;
    pcl::PointCloud<PointT>::Ptr sweepCloud1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr sweepCloud2(new pcl::PointCloud<PointT>);

    //visualization
    pcl::visualization::PCLVisualizer PkViewer, originalViewer;
    PkViewer.setCameraPosition(0,0,-3.0,0,-1,0);
    PkViewer.addCoordinateSystem(0.3);
    originalViewer.setCameraPosition(0,0,-3.0,0,-1,0);
    originalViewer.addCoordinateSystem(0.3);
    //PkViewer.addPointCloud(alignCloud);

    int line = 0;std::string filename;
    char ctmp[8];
    Eigen::Matrix4d poseTransform;
    for(int pK = 1; pK < 15; pK++)
    {

        if(pK == 1)
        {   // read the first point cloud : P0
            lastScans.reserve(16);
            for(line = 0;line < 16;line++)
            {
                sprintf(ctmp,"%d",pK);
                std::string stmp(ctmp);
                filename = dir + stmp + filesuffix[line];
                pcl::PointCloud<PointT>::Ptr scanCloud0(new pcl::PointCloud<PointT>);
                if((read2PointCloud(filename,scanCloud0) == -1))
                {
                    cout<<"Couldn't open file !\n"<<(filename).c_str()<<endl;
                    return -1;
                }
                lastScans.push_back(scanCloud0);
            }
            continue;
        }
        //read current point cloud : Pk+1
        currScans.reserve(16);
        for(line = 0;line < 16;line++)
        {
            sprintf(ctmp,"%d",pK);
            std::string stmp(ctmp);
            filename = dir + stmp + filesuffix[line];
            pcl::PointCloud<PointT>::Ptr scanCloud0(new pcl::PointCloud<PointT>);
            if((read2PointCloud(filename,scanCloud0) == -1))
            {
                cout<<"Couldn't open file !\n"<<(filename).c_str()<<endl;
                return -1;
            }
            currScans.push_back(scanCloud0);
            *sweepCloud1 += *(currScans.back());
        }

        Odometry(lastScans,currScans,poseTransform,reprojectedScans);
        lastScans = reprojectedScans;
        // Mapping
        for(line = 0; line < 16;line++)
            *sweepCloud2 += *(lastScans[line]);
 
    }
        // PkViewer.removePointCloud();
        // PkViewer.addPointCloud(sweepCloud2,filename);
        // while((!PkViewer.wasStopped())) 
        // {
        //     PkViewer.spinOnce(100);
        // }
        originalViewer.removePointCloud();
        originalViewer.addPointCloud(sweepCloud1,filename);
        while((!originalViewer.wasStopped())) 
        {
            originalViewer.spinOnce(100);
        }




// boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp test"));  //定义窗口共享指针
// int v1 ; //定义两个窗口v1，v2，窗口v1用来显示初始位置，v2用以显示配准过程
// int v2 ;
// view->createViewPort(0.0,0.0,0.5,1.0,v1);  //四个窗口参数分别对应x_min,y_min,x_max.y_max.
// view->createViewPort(0.5,0.0,1.0,1.0,v2);

// //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(srcCloud,250,0,0); //设置源点云的颜色为红色
// view->addPointCloud(srcCloud,"sources_cloud_v1",v1);
// //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color (tgtCloud,0,250,0);  //目标点云为绿色
// view->addPointCloud(tgtCloud,"target_cloud_v1",v1); //将点云添加到v1窗口

// view->setBackgroundColor(0.0,0.05,0.05,v1); //设着两个窗口的背景色
// view->setBackgroundColor(0.05,0.05,0.05,v2);

// view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sources_cloud_v1");  //设置显示点的大小
// view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v1");

// //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>aligend_cloud_color(alignCloud,255,255,255);  //设置配准结果为白色
// view->addPointCloud(alignCloud,"aligend_cloud_v2",v2);
// view->addPointCloud(tgtCloud,"target_cloud_v2",v2);

// view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"aligend_cloud_v2");
// view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v2");

// view->registerKeyboardCallback(&keyboardEvent,(void*)NULL);  //设置键盘回调函数
// int iterations = 0; //迭代次数
// while(!view->wasStopped())
// {
//         view->spinOnce(100);  //运行视图
//         //if (next_iteration)
//         {
//                 icp.align(*alignCloud);  //icp计算
//                 cout <<"has conveged:"<<icp.hasConverged()<<"score:"<<icp.getFitnessScore()<<endl;
//                 cout<<"matrix:\n"<<icp.getFinalTransformation()<<endl;
//                 cout<<"iteration = "<<++iterations;
//                 /*... 如果icp.hasConverged=1,则说明本次配准成功，icp.getFinalTransformation()可输出变换矩阵   ...*/
//                 if (iterations == 1000)  //设置最大迭代次数
//                         return 0;
//                 view->updatePointCloud(alignCloud,"aligend_cloud_v2");
                
//         }
//         next_iteration = false;  //本次迭代结束，等待触发

// }

    return (0);
}

void keyboardEvent(const pcl::visualization::KeyboardEvent &event,void *nothing)
{
    if(event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
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
