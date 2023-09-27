#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <pcl/search/impl/flann_search.hpp>
#include <sensor_msgs/PointCloud2.h> 
#include <vector>
#include "test1/dynamic_filter.h"


using namespace std;


std::string topic_pcl = "/tanwaylidar_undistort";

ros::Publisher pub_g;   // ground pointclouds
ros::Publisher pub_ng;  // non ground pointclouds
ros::Publisher pub_bg;  // background pointclouds
ros::Publisher pub_fg;  // foreground pointclouds
ros::Publisher pub_dyn; // dynamic pointclouds
ros::Publisher pub_sta; // static pointclouds

PointCloudEX::Ptr lastCloud(new PointCloudEX);
int lastFlag = 0;


/** 
 * @brief Distinguish between ground and non ground points
 * @param Cloud    the current frame
 */
void FilterGnd(const sensor_msgs::PointCloud2::ConstPtr& Cloud)
{
    // Convert point cloud types
    PointCloudEX::Ptr inCloud(new PointCloudEX);
    pcl::fromROSMsg(*Cloud, *inCloud);
    
    PointCloudEX::Ptr GndCloud(new PointCloudEX);
    PointCloudEX::Ptr noGndCloud(new PointCloudEX);

    int inNum = inCloud->size();

    int GndNum = 0;
    int noGndNum = 0;
    float dx = 0.5;
    float dy = 0.5;
    int x_len = 30;
    int y_len = 20;
    int nx = x_len / dx; //80
    int ny = 2 * y_len / dy; //10，(-y_len,y_len)
    float offx = 0, offy = -y_len;
    float THR = 0.2;
    
    // The lowest point height, highest point height, total height, average height, and number of points of each grid
    float *imgMinZ = (float*) calloc (nx*ny, sizeof(float));
    float *imgMaxZ = (float*) calloc (nx*ny, sizeof(float));
    float *imgSumZ = (float*) calloc (nx*ny, sizeof(float));
    float *imgMeanZ = (float*) calloc (nx*ny, sizeof(float));
    int *imgNumZ = (int*) calloc (nx*ny, sizeof(int));
    // Index of each point, corresponding to the grid, range [0, nx * ny)
    int *idtemp = (int*) calloc (inNum, sizeof(int)); 

    // Initialize
    for(int i = 0; i < nx * ny; i++)
    {
        imgMinZ[i] = 10;
        imgMaxZ[i] = -10;
        imgMeanZ[i] = 0;
        imgSumZ[i] = 0;
        imgNumZ[i] = 0;
    }

    for(int pid = 0; pid < inNum; pid++)
    {
        idtemp[pid] = -1;
        if(((*inCloud)[pid].x > 0) && ((*inCloud)[pid].x < x_len) && ((*inCloud)[pid].y > -y_len) && ((*inCloud)[pid].y < y_len)) //范围在(-x_len,x_len)之间
        {
            int idx = ((*inCloud)[pid].x - offx) / dx; 
            int idy = ((*inCloud)[pid].y - offy) / dy;
            idtemp[pid] = idx + idy * nx; 
            if (idtemp[pid] >= nx * ny)
                continue;
            imgSumZ[idx+idy*nx] += (*inCloud)[pid].z;
            imgNumZ[idx+idy*nx] += 1;
            if((*inCloud)[pid].z < imgMinZ[idx+idy*nx])
            {
                imgMinZ[idx+idy*nx] = (*inCloud)[pid].z;
            }
            if((*inCloud)[pid].z > imgMaxZ[idx+idy*nx]){
                imgMaxZ[idx+idy*nx] = (*inCloud)[pid].z;
            }
        }
    }
    
    // Filter out ground points
    for(int pid = 0; pid < inNum; pid++)
    {
        if (GndNum >= 60000)
            break;
        if(idtemp[pid] > 0 && idtemp[pid] < nx*ny)
        {
            imgMeanZ[idtemp[pid]] = float(imgSumZ[idtemp[pid]] / (imgNumZ[idtemp[pid]] + 0.0001));
            
            if((imgMaxZ[idtemp[pid]] - imgMeanZ[idtemp[pid]]) < THR && imgNumZ[idtemp[pid]] > 0 && imgMeanZ[idtemp[pid]] < 1)
            {
                GndCloud->push_back((*inCloud)[pid]);
                GndNum++;
            }
            else{
                noGndCloud->push_back((*inCloud)[pid]);
                noGndNum++;
            }
        }
        else{
            noGndCloud->push_back((*inCloud)[pid]);
            noGndNum++;
        }
    }
    
    free(imgMinZ);
    free(imgMaxZ);
    free(imgSumZ);
    free(imgMeanZ);
    free(imgNumZ);
    free(idtemp);

    // publish pointcloud
    sensor_msgs::PointCloud2 msg_g;
    pcl::toROSMsg(*GndCloud, msg_g);
    msg_g.header = Cloud->header;
    pub_g.publish(msg_g);

    sensor_msgs::PointCloud2 msg_ng;
    pcl::toROSMsg(*noGndCloud, msg_ng);
    msg_ng.header = Cloud->header;
    pub_ng.publish(msg_ng);
}


/** 
 * @brief Find a cluster of each point
 * @param pLabel   the label of foreground(0) and background(1)
 * @param seedId   the index of point in the pointcloud
 * @param labelId  the index of the cluster
 * @param cloud    the pointcloud to be processed
 * @param kdtree 
 * @param fSearchRadius  the search radius of kdtree
 * @param thrHeight      the thre of height
 * @return         the cluster
 */
SClusterFeature FindACluster(int *pLabel, int seedId, int labelId, 
                    PointCloudEX::Ptr cloud, pcl::KdTreeFLANN<TanwayPCLEXPoint> &kdtree, 
                    float fSearchRadius, float thrHeight,std_msgs::Header cheader)
{
    // Initialize seed
    std::vector<int> seeds;
    seeds.push_back(seedId);
    pLabel[seedId]=labelId;

    SClusterFeature cf;
    cf.pnum=1;
    cf.xmax=-2000;
    cf.xmin=2000;
    cf.ymax=-2000;
    cf.ymin=2000;
    cf.zmax=-2000;
    cf.zmin=2000;
    cf.zmean=0;

    // region growing
    while(seeds.size()>0) //The seed point found k-nearest neighbors, and then k-nearest neighbors were included in the seed point
    {
        int sid=seeds[seeds.size()-1];
        seeds.pop_back();

        TanwayPCLEXPoint searchPoint;
        searchPoint = cloud->points[sid];

        // feature statistics
        {
            if(searchPoint.x>cf.xmax) cf.xmax=searchPoint.x;
            if(searchPoint.x<cf.xmin) cf.xmin=searchPoint.x;

            if(searchPoint.y>cf.ymax) cf.ymax=searchPoint.y;
            if(searchPoint.y<cf.ymin) cf.ymin=searchPoint.y;

            if(searchPoint.z>cf.zmax) cf.zmax=searchPoint.z;
            if(searchPoint.z<cf.zmin) cf.zmin=searchPoint.z;

            cf.zmean+=searchPoint.z;
        }

        std::vector<float> k_dis;
        std::vector<int> k_inds;

        kdtree.radiusSearch(searchPoint,fSearchRadius,k_inds,k_dis);
        for(int ind=0;ind<k_inds.size();ind++)
        {
            if(pLabel[k_inds[ind]]==0)
            {
                pLabel[k_inds[ind]]=labelId;
                // cloud->points[k_inds[ind]].intensity=labelId;
                // cnum++;
                cf.pnum++;
                if(cloud->points[k_inds[ind]].z>thrHeight)
                {
                    seeds.push_back(k_inds[ind]);
                }
            }
        }
    }
    cf.zmean/=(cf.pnum+0.000001);
    return cf;
}


/** 
 * @brief Find a cluster of each point
 * @param pLabel   the label of foreground(0) and background(1)
 * @param seedId   the index of point in the pointcloud
 * @param labelId  the index of the cluster
 * @param cloud    the pointcloud to be processed
 * @param kdtree 
 * @param kdtreelast
 * @param fSearchRadius  the search radius of kdtree
 * @param thrHeight      the thre of height
 * @param cheader        the header of pointcloud
 */
void FindCluster(int *pLabel, int seedId, int labelId, 
                PointCloudEX::Ptr cloud, pcl::KdTreeFLANN<TanwayPCLEXPoint> &kdtree, 
                pcl::KdTreeFLANN<TanwayPCLEXPoint> &kdtreelast, float fSearchRadius, 
                float thrHeight,std_msgs::Header cheader)
{
    // Initialize seed
    std::vector<int> seeds;
    seeds.push_back(seedId);
    pLabel[seedId]=labelId;
    int pnum = 1;
    
    PointCloudEX::Ptr clusterCloud(new PointCloudEX);
    clusterCloud->push_back(cloud->points[seedId]);

    // region growing
    while(seeds.size()>0) //The seed point found k-nearest neighbors, and then k-nearest neighbors were included in the seed point
    {
        int sid=seeds[seeds.size()-1];
        seeds.pop_back();
        
        TanwayPCLEXPoint seedPoint;
        seedPoint = cloud->points[sid];
        std::vector<float> k_dis;
        std::vector<int> k_inds;

        kdtree.radiusSearch(seedPoint,fSearchRadius,k_inds,k_dis);
        for(int ind=0;ind<k_inds.size();ind++)
        {
            if(pLabel[k_inds[ind]]==0)
            {
                clusterCloud->push_back(cloud->points[k_inds[ind]]);
                pLabel[k_inds[ind]]=labelId;
                // cloud->points[k_inds[ind]].intensity=labelId;
                // cnum++;
                pnum++;
                if(cloud->points[k_inds[ind]].z>thrHeight)
                {
                    seeds.push_back(k_inds[ind]);
                }
            }
        }
    }
    TanwayPCLEXPoint searchPoint;
    int cnum = 0;
    for (int pid = 0; pid < pnum; pid++){
        searchPoint = clusterCloud->points[pid];
        std::vector<float> k_dis; 
        std::vector<int> k_inds; 
        kdtreelast.nearestKSearch(searchPoint, 1, k_inds, k_dis);
        if(k_dis[0] > fSearchRadius){
            cnum++;
        }
    }
    if(3 * cnum > pnum){
        for (int pid = 0; pid < cloud->points.size(); pid++){
            if(pLabel[pid] == labelId){
                pLabel[pid] = 1;
            }
        }
    }
}


/** 
 * @brief Plan 1: Find dynamic points by clustering size
 * @param cloud     the current frame
 * @param cheader   the header of pointcloud
 * @return the number of foreground points
 */
int SegObjects(PointCloudEX::Ptr cloud, std_msgs::Header cheader)
{
    int pnum = cloud->points.size();
    int labelId = 10; 
    int *pLabel=(int*)calloc(pnum, sizeof(int));
    for(int i = 0; i < pnum; i++){
        pLabel[i] = 0;
    }
    
    pcl::KdTreeFLANN<TanwayPCLEXPoint> kdtree;
    kdtree.setInputCloud (cloud);
    float fSearchRadius = 1.2;

    PointCloudEX::Ptr dynCloud(new PointCloudEX);
    PointCloudEX::Ptr staCloud(new PointCloudEX);

    //Traverse each non background point, and if the height is greater than the threshold, find a cluster and give it a index(>10)
    for(int pid = 0; pid < pnum; pid++)
    {
        if(pLabel[pid] == 0)
        {
            if(1) // threshold:cloud->points[pid].z > 0
            {
                SClusterFeature cf = FindACluster(pLabel,pid,labelId,cloud,kdtree,fSearchRadius,0,cheader);
                int isCar=0;

                float dx=cf.xmax-cf.xmin;
                float dy=cf.ymax-cf.ymin;
                float dz=cf.zmax-cf.zmin;
               
                // Requirements for filtering dynamic points
                if(cf.pnum > 5 && cf.pnum <= 200 && (dx < 2) && (dy > 1) && (dy < 2.5)){
                    isCar = 1;
                }
                if(cf.pnum > 5 && cf.pnum <= 200 && (dy < 3) && (dx > 1) && (dx < 4)){
                    isCar = 1;
                }

                if(isCar>0) // Classify into dynamic points
                {
                    for(int ii=0;ii<pnum;ii++){
                        if(pLabel[ii]==labelId){
                            pLabel[ii]=isCar;
                        }
                    }
                }
                else{
                    labelId++;
                }
            }
        }
    }
    for(int pid = 0; pid < pnum; pid++){
        if(pLabel[pid] == 1){
            dynCloud->push_back((*cloud)[pid]);
        }
        else{
            staCloud->push_back((*cloud)[pid]);
        }
    }

    std::cout << dynCloud->size() << std::endl;
    std::cout << staCloud->size() << std::endl;
    
    sensor_msgs::PointCloud2 msg_dyn;
    pcl::toROSMsg(*dynCloud, msg_dyn);
    msg_dyn.header = cheader; 
    pub_dyn.publish(msg_dyn);

    sensor_msgs::PointCloud2 msg_sta;
    pcl::toROSMsg(*staCloud, msg_sta);
    msg_sta.header = cheader;
    pub_sta.publish(msg_sta);
    return labelId-10;
}


/** 
 * @brief Plan 2: Find dynamic points by comparing with the previous frames
 * @param lastCloud   the previous frame
 * @param fgCloud     the current frame
 * @param cheader     the header of pointcloud
 */
void seg_dynamic(PointCloudEX::Ptr lastCloud, PointCloudEX::Ptr fgCloud, std_msgs::Header cheader){
    int fgNum = fgCloud->points.size(); 
    
    PointCloudEX::Ptr dynCloud(new PointCloudEX);
    PointCloudEX::Ptr staCloud(new PointCloudEX);

    int labelId = 10; 
    int *pLabel=(int*)calloc(fgNum, sizeof(int));
    for(int i = 0; i < fgNum; i++){
        pLabel[i] = 0;
    }

    pcl::KdTreeFLANN<TanwayPCLEXPoint> kdtree;
    kdtree.setInputCloud(fgCloud);
    pcl::KdTreeFLANN<TanwayPCLEXPoint> kdtreelast;
    kdtreelast.setInputCloud(lastCloud);
    int searchNum = 1;
    float fSearchRadius = 1; 
    float thrHeight = 0;
    TanwayPCLEXPoint searchPoint;
    
    for (int pid = 0; pid < fgNum; pid++){
        if(pLabel[pid] == 0){
            if(fgCloud->points[pid].z > 0)//高度阈值
            {
                FindCluster(pLabel,pid,labelId,fgCloud,kdtree,kdtreelast,fSearchRadius,thrHeight,cheader);
                // SClusterFeature cf = FindACluster(pLabel,pid,labelId,fgCloud,kdtree,kdtreelast,fSearchRadius,thrHeight,cheader);
                // int cnum = cf.pnum;
                // if(cnum>1){
                //     std::cout << "该聚类的点数为" << cnum << std::endl;
                // }
            }
        }
        
    }
    for (int pid = 0; pid < fgNum; pid++){
        if(pLabel[pid] == 1){
            dynCloud->push_back(fgCloud->points[pid]);
        }
        else{
            staCloud->push_back(fgCloud->points[pid]);
        }
    }
    
    std::cout << dynCloud->size() << std::endl;
    std::cout << staCloud->size() << std::endl;
    
    sensor_msgs::PointCloud2 msg_dyn;
    pcl::toROSMsg(*dynCloud, msg_dyn);
    msg_dyn.header = cheader; 
    pub_dyn.publish(msg_dyn);

    sensor_msgs::PointCloud2 msg_sta;
    pcl::toROSMsg(*staCloud, msg_sta);
    msg_sta.header = cheader;
    pub_sta.publish(msg_sta);
}

// 对4-6m的种子点，向下增长为背景
/** 
 * @brief Distinguish between foreground and background. For seeds with 4-6 meters height, grow downwards as the background.
 * @param Cloud     the current frame
 */
void SegBG(const sensor_msgs::PointCloud2::ConstPtr& Cloud)
{
    PointCloudEX::Ptr inCloud(new PointCloudEX);
    pcl::fromROSMsg(*Cloud, *inCloud);

    // Starting from higher points and growing from top to bottom, you can find tall backgrounds like trees and buildings
    int inNum = inCloud->points.size(); 
    std::cout << inNum << std::endl;
    int *pLabel=(int*)calloc(inNum, sizeof(int));
    
    pcl::KdTreeFLANN<TanwayPCLEXPoint> kdtree;
    kdtree.setInputCloud (inCloud);
    float fSearchRadius = 1.5;
    TanwayPCLEXPoint searchPoint;
    // if(lastFlag == 0){
    //     *lastCloud = *inCloud;
    // }
    
    PointCloudEX::Ptr bgCloud(new PointCloudEX);
    PointCloudEX::Ptr fgCloud(new PointCloudEX);

    // Initialize seed points, select all points with a height between 4-6 meters, and store the index in seeds
    std::vector<int> seeds;
    for (int pid = 0; pid < inNum; pid++)
    {
        if(inCloud->points[pid].z > 3)
        {
            pLabel[pid] = 1; // background
            if (inCloud->points[pid].z < 5)
            {
                seeds.push_back(pid);
            }
        }
        else
        {
            pLabel[pid]=0; // Temporarily classified into foreground
        }
    }

    // region growing
    while(seeds.size() > 0)
    {
        int sid = seeds[seeds.size()-1];
        seeds.pop_back();

        std::vector<float> k_dis; 
        std::vector<int> k_inds; 

        kdtree.radiusSearch(sid, fSearchRadius, k_inds, k_dis);
        
        for(int ind=0;ind<k_inds.size();ind++)
        {
            if(pLabel[k_inds[ind]]==0) // foreground=0
            {
                pLabel[k_inds[ind]]=1; // background=1
                seeds.push_back(k_inds[ind]);
            }
        }
    }
    for (int pid = 0; pid < inNum; pid++){
        if(pLabel[pid] == 1){
            bgCloud->push_back((*inCloud)[pid]);
        }
        else if(pLabel[pid] == 0){
            fgCloud->push_back((*inCloud)[pid]);
        }
    }
    
    sensor_msgs::PointCloud2 msg_bg;
    pcl::toROSMsg(*bgCloud, msg_bg);
    msg_bg.header = Cloud->header;
    pub_bg.publish(msg_bg);

    sensor_msgs::PointCloud2 msg_fg;
    pcl::toROSMsg(*fgCloud, msg_fg);
    msg_fg.header = Cloud->header;
    pub_fg.publish(msg_fg);

    // 方案一
    // int fgNum = SegObjects(fgCloud, Cloud->header);
   

    // 方案二
    // if(lastFlag == 0){
    //     lastFlag = 1;
    // }
    // else{
    //     seg_dynamic(lastCloud, fgCloud, Cloud->header);
    // }
    // lastCloud->clear();
    // lastCloud = inCloud;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "tanway_dynamic_filter");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe(topic_pcl, 10, FilterGnd);
    ros::Subscriber sub_ng = nh.subscribe("/nogndcloud", 10, SegBG);
    pub_g = nh.advertise<sensor_msgs::PointCloud2>("gndcloud", 10);
    pub_ng = nh.advertise<sensor_msgs::PointCloud2>("nogndcloud", 10);
    pub_bg = nh.advertise<sensor_msgs::PointCloud2>("bgcloud", 1000);
    pub_fg = nh.advertise<sensor_msgs::PointCloud2>("fgcloud", 1000);
    pub_dyn = nh.advertise<sensor_msgs::PointCloud2>("dyncloud", 1000);
    pub_sta = nh.advertise<sensor_msgs::PointCloud2>("stacloud", 1000);

    ros::spin();

    return 0;
}