#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp_nl.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> CloudType;
typedef pcl::PointCloud<PointT>::Ptr CloudTypePtr;
using namespace Eigen;
using namespace std;

const double cThreshold[4] = {0.0, 3e-3, 0.012,0.5};     //{planar_lower, planar_upper,edge_lower,edge_upper}
#define dEpsilon 1e-3
std::string dir = "../../data/Pk_select/";
std::string filesuffix[16] = {"_1.XYZ","_3.XYZ","_5.XYZ","_7.XYZ","_9.XYZ","_11.XYZ","_13.XYZ","_15.XYZ", \
                              "_-1.XYZ","_-3.XYZ","_-5.XYZ","_-7.XYZ","_-9.XYZ","_-11.XYZ","_-13.XYZ","_-15.XYZ"};

inline double calculate_cVal(vector<int>& S,const pcl::PointXYZRGB& centerPoint,pcl::PointCloud<PointT>::Ptr cloud);
inline double getCValue(int lo,int hi,PointT &centerPoint, CloudTypePtr cloud);
inline int setPoint2Red(pcl::PointXYZRGB& Point);
inline int setPoint2Green(pcl::PointXYZRGB& Point);
inline int setPointColor(PointT& Point,int r, int g, int b);

int read2PointCloud(std::string& file2read,pcl::PointCloud<PointT>::Ptr cloud);
int read2PointCloud(std::string file2read,pcl::PointCloud<PointT>& cloud);

inline int findCIndex(std::vector<double>& cValue, double c);
inline void getPointDistanceSubregion(CloudTypePtr &cloud, vector<int>& S, PointT querypoint,vector<float>& distance);
inline int elimiAbnormalPoint(vector<int>& S,vector<float>& distance,pcl::PointCloud<PointT>::Ptr cloud);


inline double calcPoint2PointDis(PointT& p1,PointT& p2);

int extractFeatruePoints(pcl::PointCloud<PointT>::Ptr cloud,std::vector<int>&edge_index,std::vector<int>&plane_index);
inline int isEdgeorPlanePoint(vector<double>&cVal,vector<int>&edge_index, \
                              vector<int>&plane_index,const double threshold[4],pcl::PointCloud<PointT>::Ptr cloud);

//B. Find the feature point correspondence
inline int FindCorrespondence(int sweepindex, std::vector<CloudTypePtr>& lastScans, std::vector<CloudTypePtr>& currScans, \
                             std::vector<vector<int> >& EdgePointIndices, std::vector<vector<int> >& PlanarPointIndices,\
                             CloudTypePtr srcCloud, CloudTypePtr tgtCloud);
inline int isEdgePoint(CloudTypePtr scan, int index, float& cValue);
inline int isPlanarPoint(CloudTypePtr scan,int index,float& cValue);
double getEdgePoint2CorrespondenceDistance(PointT i,PointT j, PointT l);
double getPlanarPoint2CorrespondenceDistance(PointT i,PointT j,PointT l,PointT m);
double Vector3dModulus(Eigen::Vector3d vec);

PointT getEdgePoint2LineFootpoint(PointT& i, PointT& j, PointT& l);
PointT getPlanarPoint2PlaneFootpoint(PointT& i, PointT& j, PointT& l, PointT& m);

//D. Odometry
void Odometry(std::vector<CloudTypePtr>& lastScans, std::vector<CloudTypePtr>& currentScans, Eigen::Matrix4d& poseTransform, \
              std::vector<CloudTypePtr>& reprojectedScans);

void keyboardEvent(const pcl::visualization::KeyboardEvent &event,void *nothing);

struct CorrespondenceInfo{
    int featurepoint_index;
    int correspondence_index;
    double distance;
};

int extractFeatruePoints(pcl::PointCloud<PointT>::Ptr cloud,std::vector<int>&edge_index,std::vector<int>&plane_index)
{
  int cloudsize = cloud->width * cloud->height;
  int coefK = cloudsize>>7;
  //printf("coefK:%d\n",coefK);

  std::vector<double> cVal(cloudsize,0);
  //create a “searchPoint” which is assigned random coordinates
  PointT searchPoint;
  //To get c value
  double c;int low, high;
  for(int cnt=0;cnt < cloudsize;++cnt)
  {
      searchPoint = cloud->points[cnt];
      low  = (cnt - coefK + cloudsize)%cloudsize;
      high = (cnt + coefK)%cloudsize;
      c = getCValue(low,high,searchPoint,cloud);
      cVal[cnt] = c;
  }

  isEdgeorPlanePoint(cVal,edge_index,plane_index,cThreshold,cloud);
  for(int i=0;i<edge_index.size();++i)
  {
      setPoint2Red(cloud->points[edge_index[i]]);
  }
  for(int i=0;i<plane_index.size();++i)
  {
      setPoint2Green(cloud->points[plane_index[i]]);
  }
  return 0;
}

/*
 * Argument: lo, hi
 * S[lo,hi]
 */
inline double getCValue(int lo,int hi,PointT &centerPoint, CloudTypePtr cloud)
{
  double cValue;
  int cloudsize = cloud->width * cloud->height;
  if(lo < 0 || hi > cloudsize)
    return -1;
  double D_centerPoint=sqrt((centerPoint.x)*(centerPoint.x)+(centerPoint.y)*(centerPoint.y)+(centerPoint.z)*(centerPoint.z));
  pcl::PointXYZRGB sumPoint(0,0,0);
  double D_sum=0;
  for(int i=lo;i != hi; i = (i+1)%cloudsize)
  {
      sumPoint.x+=(centerPoint.x-cloud->points[i].x);
      sumPoint.y+=(centerPoint.y-cloud->points[i].y);
      sumPoint.z+=(centerPoint.z-cloud->points[i].z);
  }
  D_sum=sqrt((sumPoint.x)*(sumPoint.x)+(sumPoint.y)*(sumPoint.y)+(sumPoint.z)*(sumPoint.z));
  cValue = D_sum/((abs(hi - lo))*D_centerPoint);
  return cValue;
}

inline double calcPoint2PointDis(PointT& p1,PointT& p2)
{
    double dis;
    double dx=p1.x-p2.x;
    double dy=p1.y-p2.y;
    double dz=p1.z-p2.z;
    dis=sqrt(dx*dx+dy*dy+dz*dz);
    return dis;
}

inline int isEdgeorPlanePoint(vector<double>&cVal,vector<int>&edge_index,\
        vector<int>&plane_index,const double threshold[4],pcl::PointCloud<PointT>::Ptr cloud)
{
/*    int csize = cVal.size();
    double P2PDistance = 0;
    double CMax[2]={0}, CMin[4]={-1}; int CMaxInd[2]={-1}, CMinInd[4]={-1};
    for(int iterCount = 0; iterCount < csize; iterCount++)
    {
      //extract 2 edge points & 4 planar points from subregion
      P2PDistance = calcPoint2PointDis(cloud->points[iterCount],cloud->points[iterCount+1]);
      printf("P2PDistance:%f\n",P2PDistance);
      if(P2PDistance > 0.5)
        continue;
    }*/
        int edge_beg=-1;
        int plane_beg=-1;
        int edge_prev=0;
        int plane_prev=0;
        int edge_max=-1;
        int plane_max=-1;
        for(int i=0;i<cVal.size();++i)
        {
            if(cVal[i]>threshold[2]&&cVal[i]<threshold[3])
            {
    #if 1
                if(edge_beg==-1)        //第一次发现角点
                {
                    edge_beg=i;
                    edge_prev=i;
                    edge_max=i;
                }
                if(i-edge_prev<=2)      //认为仍然是连在一起的
                {
                    if(cVal[i]>cVal[edge_max])
                        edge_max=i;
                    edge_prev=i;
                }
    #else
                edge_index.push_back(i);
    #endif
            }
            else if(cVal[i]>threshold[0]&&cVal[i]<threshold[1])
            {
                if(i-edge_prev>2&&edge_beg!=-1)       //出现角点中断
                {
                    edge_index.push_back(edge_max);
                    edge_beg=-1;
                }
                plane_index.push_back(i);
            }
        }
    #if 1
        int edge_cnt=0;
        for(int i=edge_index.size()-1;i>=0;--i)
        {
            int tmp_index=edge_index[i];
            for(int j=tmp_index-4;j<tmp_index+4;++j)
            {
                double tmp_dis=calcPoint2PointDis(cloud->points[tmp_index],cloud->points[j]);
    //            std::cout<<tmp_dis<<endl;
                if(tmp_dis>0.05)
                {
                    ++edge_cnt;
                }
            }
            if(edge_cnt>4)
            {
                edge_index.pop_back();
            }
            edge_cnt=0;

        }
    #endif
        // std::cout<<"plane_index size: "<<plane_index.size()<<endl;
        // std::cout<<"edge_index size: "<<edge_index.size()<<endl;
        return 0;

}

//return true or false
inline int isEdgePoint(CloudTypePtr scan, int index, float& cValue)
{
  int scansize = scan->width * scan->height;
  int coefK = scansize>>7;
  PointT searchPoint;

  if(index > scansize)
    return 0;
  //To get c value
  double c;int low, high;
  searchPoint = scan->points[index];
  low  = (index - coefK + scansize)%scansize;
  high = (index + coefK)%scansize;
  c = getCValue(low,high,searchPoint,scan);
  cValue = c;
  //is it a edge point ?
  if(c > cThreshold[2] && c < cThreshold[3])
    return 1;
  else
    return 0;
}

//compute the distance from edge point i to its correspondence
//edge point i; edge line {j,l}
double getEdgePoint2CorrespondenceDistance(PointT i,PointT j, PointT l)
{
    //typedef Matrix< double , 3 , 1> Eigen::Vector3d
  Eigen::Vector3d Xi(i.x,i.y,i.z), Xj(j.x,j.y, j.z), Xl(l.x,l.y,l.z);
  Eigen::Vector3d Xij, Xil, Xjl, numerator,dinominator;
  Xij = Xi - Xj;
  Xil = Xi - Xl;
  Xjl = Xj - Xl;
  numerator = Xij.cross(Xil);
  dinominator = Xjl;
  double distance = Vector3dModulus(numerator) / Vector3dModulus(dinominator);
  return distance;
}

PointT getEdgePoint2LineFootpoint(PointT& i, PointT& j, PointT& l)
{
    PointT footpoint;
    double m,n,p,t;
    m = j.x - l.x;
    n = j.y - l.y;
    p = j.z - l.z;
    t = (m*(i.x - j.x) + n*(i.y - j.y) + p*(i.z - j.z))/(m*m + n*n + p*p);
    footpoint.x = m*t + j.x;
    footpoint.y = n*t + j.y;
    footpoint.z = p*t + j.z;
    return footpoint;
}

inline int isPlanarPoint(CloudTypePtr scan,int index,float& cValue)
{
    int scansize = scan->width * scan->height;
    int coefK = scansize>>7;
    PointT searchPoint;
    if(index > scansize)
        return 0;
    //To get c value
    double c;int low, high;
    searchPoint = scan->points[index];
    low  = (index - coefK + scansize)%scansize;
    high = (index + coefK) % scansize;
    c = getCValue(low,high,searchPoint,scan);
    cValue = c;
    //is it a planar point ?
    if(c > cThreshold[0] && c < cThreshold[1])
        return 1;
    else
        return 0;   
}

double getPlanarPoint2CorrespondenceDistance(PointT i,PointT j,PointT l,PointT m)
{
    Eigen::Vector3d Xi(i.x,i.y,i.z),Xj(j.x,j.y,j.z),Xl(l.x,l.y,l.z),Xm(m.x,m.y,m.z);
    Eigen::Vector3d Xij,Xjl,Xjm,dinominator;
    double numerator = 0;
    Xij = Xi - Xj;
    Xjl = Xj - Xl;
    Xjm = Xj - Xm;
    numerator = abs(Xij.dot(Xjl.cross(Xjm)));
    dinominator = Xjl.cross(Xjm);
    return (numerator/Vector3dModulus(dinominator));
}

PointT getPlanarPoint2PlaneFootpoint(PointT& i, PointT& j, PointT& l, PointT& m)
{
    //establish the plane equation
    //the plane is composed of three non-coplanar point j, l, m
    double A, B, C, D = 1;
    Eigen::Matrix3d coefMatrix, leftMat; 
    Eigen::Vector3d coefPlane, rightMat, footpointVector;
    PointT footpoint;

    coefMatrix << j.x, j.y, j.z, 
                  l.x, l.y, l.z, 
                  m.x, m.y, m.z;
    coefPlane = -(coefMatrix.inverse())*(Eigen::VectorXd::Ones(3));
    leftMat << (l.x - j.x), (l.y - j.y), (l.z - j.z),                          //x2 - x1, y2 - y1, z2 - z1
               (m.x - j.x), (m.y - j.y), (m.z - j.z),                          //x3 - x1, y3 - y1, z3 - z1
               (coefPlane(0)), (coefPlane(1)), (coefPlane(2));                 //   A   ,    B   ,    C

    rightMat << (i.x*(l.x - j.x) + i.y*(l.y - j.y) + i.z*(l.z - j.z)),         //x4(x2 - x1)+y4(y2 - y1)+z4(z2 - z1)
                (i.x*(m.x - j.x) + i.y*(m.y - j.y) + i.z*(m.z - j.z)),         //x4(x3 - x1)+y4(y3 - y1)+z4(z3 - z1)
                -1;                                                            //-1
    footpointVector = leftMat.inverse() * rightMat;
    
    footpoint.x = footpointVector(0);
    footpoint.y = footpointVector(1);
    footpoint.z = footpointVector(2);
    return footpoint; 
}

double Vector3dModulus(Eigen::Vector3d vec)
{
    return sqrt(vec(0)*vec(0) + vec(1)*vec(1) + vec(2)*vec(2));
}


inline int 
FindCorrespondence(int sweepindex,std::vector<CloudTypePtr>& lastScans, std::vector<CloudTypePtr>& currScans, \
                             std::vector<vector<int> >& EdgePointIndices, std::vector<vector<int> >& PlanarPointIndices, \
                             CloudTypePtr srcCloud, CloudTypePtr tgtCloud)
{
//   if(sweepindex == 0)
//     poseTransform = 0;
  pcl::KdTreeFLANN<PointT> KdTreeScanj,KdTreeScanl;
  int pscanj, pscanl;
  for(int line = 0;line < 16;line++)
  {
    if(line < 15)
    {
        pscanj = line;
        pscanl = line+1;
    }
    else
    {
        pscanj = line;
        pscanl = line-1;
    }
    KdTreeScanj.setInputCloud(lastScans[pscanj]);
    KdTreeScanl.setInputCloud(lastScans[pscanl]);
    //cout<<"line:"<<line<<endl;
    int cntedge = 0;
    //compute the distance from edge point i to edge line 
    for(int pI = 0;pI < EdgePointIndices[line].size(); pI++)
    {
        CloudTypePtr scani = currScans[line];
        int i_index = EdgePointIndices[line][pI];       //cout<<"Edge_i_index:"<<i_index<<endl;
        std::vector<int> j_indices,l_indices;
        std::vector<float> j_sqrdistance,l_sqrdistance;
        KdTreeScanj.nearestKSearch(scani->points[i_index],1,j_indices,j_sqrdistance);
        KdTreeScanl.nearestKSearch(scani->points[i_index],1,l_indices,l_sqrdistance);
        //Verify whether the point j & l is edge points
        float j_cValue,l_cValue;                        //cout<<"Edge_j_index:"<<j_indices[0]<<endl<<"Edge_l_index:"<<l_indices[0]<<endl;
        if((isEdgePoint(lastScans[pscanj],j_indices[0],j_cValue) == 1) && (isEdgePoint(lastScans[pscanl],l_indices[0],l_cValue) == 1))
        {
            PointT footpoint;
            cntedge ++;
            CloudTypePtr scanj,scanl;
            scanj = lastScans[pscanj];
            scanl = lastScans[pscanl];
            srcCloud->push_back((currScans[line])->points[i_index]);    
            footpoint = getEdgePoint2LineFootpoint(scani->points[i_index], \
                                scanj->points[j_indices[0]], scanl->points[l_indices[0]]);      
            tgtCloud->push_back(footpoint);     
            setPointColor(srcCloud->back(),128,255,0); 
            setPointColor(tgtCloud->back(),255,255,0);
            //cout<<"The edge points'["<<cntedge<<"] footpoint:\n"<<footpoint<<endl;
            double distance = 0;
            distance = getEdgePoint2CorrespondenceDistance(scani->points[i_index], scanj->points[j_indices[0]],scanl->points[l_indices[0]]);   
            if(distance < dEpsilon)
                cout<<"The edge points'["<<cntedge<<"] distance:"<<endl<<distance<<endl;
        }else continue;
    }
    int cntplanar = 0;
    //compute the distance from planar point to planar patches
    for(int pI = 0;pI < PlanarPointIndices[line].size();pI++)
    {
        CloudTypePtr scani = currScans[line];
        int i_index = PlanarPointIndices[line][pI];     //cout<<"Planar_i_index:"<<i_index<<endl;
        std::vector<int> j_indices,l_indices;
        std::vector<float> j_sqrdistance,l_sqrdistance;
        KdTreeScanj.nearestKSearch(scani->points[i_index],1,j_indices,j_sqrdistance);
        KdTreeScanl.nearestKSearch(scani->points[i_index],2,l_indices,l_sqrdistance);
        float j_cValue,l_cValue,m_cValue;              // cout<<"Planar_j_index:"<<j_indices[0]<<endl<<"Planar_l_index:"<<l_indices[0]<<endl;
        //Verify whether the point j & l is planar points
        if((isPlanarPoint(lastScans[pscanj],j_indices[0],j_cValue) == 1) && \
           (isPlanarPoint(lastScans[pscanl],l_indices[0],l_cValue) == 1) && \
           (isPlanarPoint(lastScans[pscanl],l_indices[1],m_cValue) == 1))
           {
               PointT footpoint;
               cntplanar ++;
               CloudTypePtr scanj,scanl,scanm;
               scanj = lastScans[pscanj];
               scanl = lastScans[pscanl];
               scanm = lastScans[pscanl];
               srcCloud->push_back((currScans[line])->points[i_index]); 
               footpoint =  getPlanarPoint2PlaneFootpoint( \
               scani->points[i_index],scanj->points[j_indices[0]],scanl->points[l_indices[0]],scanm->points[l_indices[1]]);        
               tgtCloud->push_back(footpoint);
               setPointColor(srcCloud->back(),128,255,0); 
               setPointColor(tgtCloud->back(),255,255,0); 
               // cout<<"The planar points'["<<cntplanar<<"] footpoint:"<<endl<<footpoint<<endl;
                double distance = 0;              
                distance = getPlanarPoint2CorrespondenceDistance( \
                scani->points[i_index],scanj->points[j_indices[0]],scanl->points[l_indices[0]],scanm->points[l_indices[1]]);
                if(distance < dEpsilon)
                    cout<<"The planar points'["<<cntplanar<<"] distance:"<<endl<<distance<<endl;
           }else continue;
    }
  }
}

void Odometry(std::vector<CloudTypePtr>& lastScans, std::vector<CloudTypePtr>& currentScans, Eigen::Matrix4d& poseTransform, \
              std::vector<CloudTypePtr>& reprojectedScans)
{
    CloudTypePtr srcCloud(new pcl::PointCloud<PointT>), tgtCloud(new pcl::PointCloud<PointT>), alignCloud(new pcl::PointCloud<PointT>);
    
    //copy the currentScans
    std::vector<CloudTypePtr> iterScans;
    for(int line = 0;line < 16; line++)
    {
        CloudTypePtr pscan(new pcl::PointCloud<PointT>);
        *pscan = *(currentScans[line]);
        iterScans.push_back(pscan);
    }
  
    pcl::IterativeClosestPointNonLinear<PointT,PointT> icp;    
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(5);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(0.01);

    Eigen::Matrix4d nextTransform;
    int icpMaximumIterations = 5;

    //extract feature points from Pk+1
    std::vector<vector<int> > EdgePointIndices(16);
    std::vector<vector<int> > PlanarPointIndices(16);
    for(int line = 0;line < 16;line++)
    {
        extractFeatruePoints(iterScans[line],EdgePointIndices[line],PlanarPointIndices[line]);
    }

    for(int iter = 0; iter < icpMaximumIterations; iter++)
    {
        //reproject the Ek+1 & Hk+1 to time tk+1
        if(iter != 0)
        {
            Eigen::Vector4d tmpPoint;
            PointT currpoint;
            for(int line = 0 ;line < 16;line++)
            {
                for(int edge_iter = 0; edge_iter < EdgePointIndices[line].size();edge_iter++)
                {
                    currpoint = iterScans[line]->points[EdgePointIndices[line][edge_iter]];
                    tmpPoint<<currpoint.x, currpoint.y, currpoint.z, 1;
                    tmpPoint = nextTransform.inverse() * tmpPoint;
                    currpoint.x = tmpPoint(0);  currpoint.y = tmpPoint(1);  currpoint.z = tmpPoint(2);
                    iterScans[line]->points[EdgePointIndices[line][edge_iter]] = currpoint;
                }
                for(int planar_iter = 0; planar_iter < PlanarPointIndices[line].size();planar_iter++)
                {
                    currpoint = iterScans[line]->points[PlanarPointIndices[line][planar_iter]];
                    tmpPoint<<currpoint.x, currpoint.y, currpoint.z, 1;
                    tmpPoint = nextTransform.inverse() * tmpPoint;
                    currpoint.x = tmpPoint(0);  currpoint.y = tmpPoint(1);  currpoint.z = tmpPoint(2);
                    iterScans[line]->points[PlanarPointIndices[line][planar_iter]] = currpoint;                
                }
            }
        }
        else  // linearly interpolate once
        {
            PointT currpoint;   double interpolation = 0;
            poseTransform = Eigen::Matrix4d::Identity(4,4);
            for(int line = 0 ;line < 16;line++)
            {
                for(int edge_iter = 0; edge_iter < EdgePointIndices[line].size();edge_iter++)
                {
                    interpolation = (EdgePointIndices[line][edge_iter]+1)*1.0f/iterScans[line]->size();
                    currpoint = iterScans[line]->points[EdgePointIndices[line][edge_iter]];
                    currpoint.x /= interpolation;  currpoint.y /= interpolation;  currpoint.z /= interpolation;
                    iterScans[line]->points[EdgePointIndices[line][edge_iter]] = currpoint;
                }
                for(int planar_iter = 0; planar_iter < PlanarPointIndices[line].size();planar_iter++)
                {
                    interpolation = (PlanarPointIndices[line][planar_iter]+1)*1.0f/iterScans[line]->size();
                    currpoint = iterScans[line]->points[PlanarPointIndices[line][planar_iter]];
                    currpoint.x /= interpolation;  currpoint.y /= interpolation;  currpoint.z /= interpolation;
                    iterScans[line]->points[PlanarPointIndices[line][planar_iter]] = currpoint;                
                }
            }           
        }
        //find feature point correspondence
        FindCorrespondence(0,lastScans,iterScans,EdgePointIndices,PlanarPointIndices,srcCloud,tgtCloud);
        icp.setInputSource(srcCloud);   //srcCloud
        icp.setInputTarget(tgtCloud);   //tgtCloud
        icp.align(*alignCloud);

        //Update Tk+1 for a nonlinear iteration
        nextTransform = (icp.getFinalTransformation()).cast<double>();
        //accumulate the pose transform
        poseTransform = poseTransform * nextTransform;          
        cout<<"matrix"<<iter<<":\n"<<icp.getFinalTransformation()<<endl;
        // if(icp.hasConverged())
        // {
        //     cout <<"has conveged:\n"<<"score:"<<icp.getFitnessScore()<<endl;
        //     break;
        // }
    }

    reprojectedScans.reserve(16);
    // reproject the currentScans to tk+2
    for(int line = 0;line < 16;line++)
    {
        Eigen::Vector4d tmpPoint;
        PointT currpoint;
        CloudTypePtr scan(new pcl::PointCloud<PointT>);
        for(int iter = 0; iter < (currentScans[line])->size();iter++)
        {
            currpoint = currentScans[line]->points[iter];
            tmpPoint<<currpoint.x, currpoint.y, currpoint.z, 1;
            //tmpPoint = (currentScans[line]->size() * 1.0f / (iter+1)) * nextTransform.inverse() * tmpPoint;
            tmpPoint = nextTransform.inverse() * tmpPoint;
            currpoint.x = tmpPoint(0);  currpoint.y = tmpPoint(1);  currpoint.z = tmpPoint(2);
            //currentScans[line]->points[iter] = currpoint;            
           // (reprojectedScans[line])->push_back(currpoint);
           scan->push_back(currpoint);
        }
        reprojectedScans.push_back(scan);
    }
    //poseTransform = nextTransform;   
    cout<<"pose Transform:\n"<<poseTransform<<endl;
}

class LidarOdometry{
    public:
    private:
};
#endif
