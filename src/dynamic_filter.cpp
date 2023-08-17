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

ros::Publisher pub_g;
ros::Publisher pub_ng;
ros::Publisher pub_bg;
ros::Publisher pub_fg;


// 对4-6m的种子点，向下增长为背景
void SegBG(const sensor_msgs::PointCloud2::ConstPtr& Cloud)
{
    PointCloudEX::Ptr inCloud(new PointCloudEX);
    pcl::fromROSMsg(*Cloud, *inCloud);

    //从较高的一些点开始从上往下增长，这样能找到一些类似树木、建筑物这样的高大背景
    int inNum = inCloud->points.size(); // 输入点云的点数
    std::cout << inNum << std::endl;
    int *pLabel=(int*)calloc(inNum, sizeof(int));
    //调用pcl的kdtree生成方法
    pcl::KdTreeFLANN<TanwayPCLEXPoint> kdtree;
    kdtree.setInputCloud (inCloud);
    float fSearchRadius = 1.5;
    TanwayPCLEXPoint searchPoint;

    // 背景点云和前景点云
    PointCloudEX::Ptr bgCloud(new PointCloudEX);
    PointCloudEX::Ptr fgCloud(new PointCloudEX);

    // 初始化种子点，选取高度在4-6m之间的所有点，将索引存入seeds
    std::vector<int> seeds;
    for (int pid = 0; pid < inNum; pid++)
    {
        if(inCloud->points[pid].z > 4)
        {
            pLabel[pid] = 1; // 背景
            if (inCloud->points[pid].z < 6)
            {
                seeds.push_back(pid);
            }
        }
        else
        {
            pLabel[pid]=0; // 暂时归进前景
        }
    }

    //区域增长
    while(seeds.size() > 0)
    {
        int sid = seeds[seeds.size()-1]; // 从后往前依次增长
        seeds.pop_back();

        std::vector<float> k_dis; // 存储近邻点对应距离的平方
        std::vector<int> k_inds; // 存储查询近邻点索引

        // 查询point半径为radius邻域球内的点，搜索结果默认是按照距离point点的距离从近到远排序
        if (inCloud->points[sid].x < 44.8) // 限制了x的范围，对于近点和远点采用不同的半径
            kdtree.radiusSearch(sid, fSearchRadius, k_inds, k_dis);
        else
            kdtree.radiusSearch(sid, 1.5*fSearchRadius, k_inds, k_dis);

        for(int ind=0;ind<k_inds.size();ind++)
        {
            if(pLabel[k_inds[ind]]==0) // 前景=0
            {
                pLabel[k_inds[ind]]=1; // 归到背景=1
                if(inCloud->points[k_inds[ind]].z>0.2)//（防止增长到了近地面）地面20cm以下不参与背景分割，以防止错误的地面点导致过度分割
                {
                    seeds.push_back(k_inds[ind]);
                }
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
    // 发布点云
    std::cout << bgCloud->size() << std::endl;
    std::cout << fgCloud->size() << std::endl;

    sensor_msgs::PointCloud2 msg_bg;
    pcl::toROSMsg(*bgCloud, msg_bg);
    msg_bg.header = Cloud->header;
    pub_bg.publish(msg_bg);

    sensor_msgs::PointCloud2 msg_fg;
    pcl::toROSMsg(*fgCloud, msg_fg);
    msg_fg.header = Cloud->header;
    pub_fg.publish(msg_fg);
}

// 区分地面点和非地面点
// int FilterGndForPos_cor(float* GndPoints, float* noGndPoints,float*inPoints,int inNum)
// {
//     // 数值待修改
//     int GndNum = 0;
//     int noGndNum = 0;
//     float dx = 2;
//     float dy = 2;
//     int x_len = 20;
//     int y_len = 10;
//     int nx = 2 * x_len / dx; //80，乘2是因为范围在(-x_len,x_len)之间
//     int ny = 2 * y_len / dy; //10
//     float offx = -20, offy = -10;
//     float THR = 0.4;
    
//     // 存每个栅格的最低点高度、最高点高度、高度总和、高度均值、点数
//     float *imgMinZ = (float*) calloc (nx*ny, sizeof(float));
//     float *imgMaxZ = (float*) calloc (nx*ny, sizeof(float));
//     float *imgSumZ = (float*) calloc (nx*ny, sizeof(float));
//     float *imgMeanZ = (float*) calloc (nx*ny, sizeof(float));
//     int *imgNumZ = (int*) calloc (nx*ny, sizeof(int));
//     // 存每个点的索引，对应到栅格，范围[0,nx*ny)
//     int *idtemp = (int*) calloc (inNum, sizeof(int)); 
//     // 初始化
//     for(int i = 0; i < nx * ny; i++)
//     {
//         imgMinZ[i] = 10;
//         imgMaxZ[i] = -10;
//         imgMeanZ[i] = 0;
//         imgSumZ[i] = 0;
//         imgNumZ[i] = 0;
//     }

//     // 遍历输入点云的每个点
//     for(int pid = 0; pid < inNum; pid++)
//     {
//         idtemp[pid] = -1;
//         // 获得每个栅格的最低高度、最高高度、点数、高度总和
//         if((inPoints[pid*4] > -x_len) && (inPoints[pid*4] < x_len) && (inPoints[pid*4+1] > -y_len) && (inPoints[pid*4+1] < y_len)) //范围在(-x_len,x_len)之间
//         {
//             int idx = (inPoints[pid*4] - offx) / dx; // 点云坐标的存储方式可能需要修改
//             int idy = (inPoints[pid*4+1] - offy) / dy;
//             idtemp[pid] = idx + idy * nx; //获得该点的索引
//             if (idtemp[pid] >= nx * ny)
//                 continue;
//             imgSumZ[idx+idy*nx] += inPoints[pid*4+2];
//             imgNumZ[idx+idy*nx] += 1;
//             if(inPoints[pid*4+2] < imgMinZ[idx+idy*nx])
//             {
//                 imgMinZ[idx+idy*nx] = inPoints[pid*4+2];
//             }
//             if(inPoints[pid*4+2] > imgMaxZ[idx+idy*nx]){
//                 imgMaxZ[idx+idy*nx] = inPoints[pid*4+2];
//             }
//         }
//     }
//     // 筛选出地面点：考虑以点云的形式输出地面点和非地面点，便于后续处理
//     for(int pid = 0; pid < inNum; pid++)
//     {
//         if (GndNum >= 60000)
//             break;
//         if(idtemp[pid] > 0 && idtemp[pid] < nx*ny)
//         {
//             imgMeanZ[idtemp[pid]] = float(imgSumZ[idtemp[pid]] / (imgNumZ[idtemp[pid]] + 0.0001));
//             //地面点条件：最高点与均值高度差小于阈值；点数大于3；均值高度小于1 
//             if((imgMaxZ[idtemp[pid]] - imgMeanZ[idtemp[pid]]) < THR && imgNumZ[idtemp[pid]] > 3 && imgMeanZ[idtemp[pid]] < 2)
//             {
//                 GndPoints[GndNum*4]=inPoints[pid*4];
//                 GndPoints[GndNum*4+1]=inPoints[pid*4+1];
//                 GndPoints[GndNum*4+2]=inPoints[pid*4+2];
//                 GndPoints[GndNum*4+3]=inPoints[pid*4+3];
//                 GndNum++;
//             }
//             else{
//                 noGndPoints[noGndNum*4]=inPoints[pid*4]; 
//                 noGndPoints[noGndNum*4+1]=inPoints[pid*4+1];
//                 noGndPoints[noGndNum*4+2]=inPoints[pid*4+2];
//                 noGndPoints[noGndNum*4+3]=inPoints[pid*4+3];  
//             }
//         }
//     }
//     // 释放内存
//     free(imgMinZ);
//     free(imgMaxZ);
//     free(imgSumZ);
//     free(imgMeanZ);
//     free(imgNumZ);
//     free(idtemp);
//     return GndNum;
// }

// 区分地面点和非地面点(输入输出改成点云形式)
// int FilterGnd(PointCloudEX::Ptr& GndCloud,  PointCloudEX::Ptr& noGndCloud, sensor_msgs::PointCloud2::Ptr& Cloud, int inNum)
void FilterGnd(const sensor_msgs::PointCloud2::ConstPtr& Cloud)
{
    // 输入点云类型转到tanway
    PointCloudEX::Ptr inCloud(new PointCloudEX);
    pcl::fromROSMsg(*Cloud, *inCloud);
    // 地面点云和非地面点云
    PointCloudEX::Ptr GndCloud(new PointCloudEX);
    PointCloudEX::Ptr noGndCloud(new PointCloudEX);

    int inNum = inCloud->size();
    // float fSearchRadius = 0.5;
    //调用SegBG
    // int *pLabel=(int*)calloc(inNum, sizeof(int));
    //调用pcl的kdtree生成方法
    // pcl::KdTreeFLANN<TanwayPCLEXPoint> kdtree;
    // kdtree.setInputCloud (inCloud);
    // SegBG(inCloud, kdtree, fSearchRadius);
    // 数值待修改 
    int GndNum = 0;
    int noGndNum = 0;
    float dx = 1;
    float dy = 1;
    int x_len = 300;
    int y_len = 245;
    int nx = x_len / dx; //80
    int ny = 2 * y_len / dy; //10，乘2是因为范围在(-y_len,y_len)之间
    float offx = 0, offy = -y_len;
    float THR = 0.2;
    
    // 存每个栅格的最低点高度、最高点高度、高度总和、高度均值、点数
    float *imgMinZ = (float*) calloc (nx*ny, sizeof(float));
    float *imgMaxZ = (float*) calloc (nx*ny, sizeof(float));
    float *imgSumZ = (float*) calloc (nx*ny, sizeof(float));
    float *imgMeanZ = (float*) calloc (nx*ny, sizeof(float));
    int *imgNumZ = (int*) calloc (nx*ny, sizeof(int));
    // 存每个点的索引，对应到栅格，范围[0,nx*ny)
    int *idtemp = (int*) calloc (inNum, sizeof(int)); 


    // 初始化
    for(int i = 0; i < nx * ny; i++)
    {
        imgMinZ[i] = 10;
        imgMaxZ[i] = -10;
        imgMeanZ[i] = 0;
        imgSumZ[i] = 0;
        imgNumZ[i] = 0;
    }

    // 遍历输入点云的每个点
    for(int pid = 0; pid < inNum; pid++)
    {
        idtemp[pid] = -1;
        // 获得每个栅格的最低高度、最高高度、点数、高度总和
        if(((*inCloud)[pid].x > 0) && ((*inCloud)[pid].x < x_len) && ((*inCloud)[pid].y > -y_len) && ((*inCloud)[pid].y < y_len)) //范围在(-x_len,x_len)之间
        {
            int idx = ((*inCloud)[pid].x - offx) / dx; // 点云坐标的存储方式可能需要修改
            int idy = ((*inCloud)[pid].y - offy) / dy;
            idtemp[pid] = idx + idy * nx; //获得该点的索引
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
    // std::cout << MaxX << ", " << MinX << ", " << MaxY << ", " << MinY << ", " << MaxY-MinY << std::endl;
    // std::cout << aMaxX << ", " << aMaxY << ", " << aMinY << std::endl;
    // 筛选出地面点：考虑以点云的形式输出地面点和非地面点，便于后续处理
    for(int pid = 0; pid < inNum; pid++)
    {
        if (GndNum >= 60000)
            break;
        if(idtemp[pid] > 0 && idtemp[pid] < nx*ny)
        {
            imgMeanZ[idtemp[pid]] = float(imgSumZ[idtemp[pid]] / (imgNumZ[idtemp[pid]] + 0.0001));
            //地面点条件：最高点与均值高度差小于阈值；点数大于3；均值高度小于1 
            if((imgMaxZ[idtemp[pid]] - imgMeanZ[idtemp[pid]]) < THR && imgNumZ[idtemp[pid]] > 3 && imgMeanZ[idtemp[pid]] < 1)
            {
                GndCloud->push_back((*inCloud)[pid]);
                GndNum++;
            }
            else{
                noGndCloud->push_back((*inCloud)[pid]);
                noGndNum++;
            }
        }
    }
    // 释放内存
    free(imgMinZ);
    free(imgMaxZ);
    free(imgSumZ);
    free(imgMeanZ);
    free(imgNumZ);
    free(idtemp);
    // std::cout << GndNum << std::endl;
    // 发布点云
    sensor_msgs::PointCloud2 msg_g;
    pcl::toROSMsg(*GndCloud, msg_g);
    msg_g.header = Cloud->header;
    pub_g.publish(msg_g);

    sensor_msgs::PointCloud2 msg_ng;
    pcl::toROSMsg(*noGndCloud, msg_ng);
    msg_ng.header = Cloud->header;
    pub_ng.publish(msg_ng);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tanway_dynamic_filter");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe(topic_pcl, 10, SegBG);
    pub_g = nh.advertise<sensor_msgs::PointCloud2>("gndcloud", 10);
    pub_ng = nh.advertise<sensor_msgs::PointCloud2>("nogndcloud", 10);
    pub_bg = nh.advertise<sensor_msgs::PointCloud2>("bgcloud", 1000);
    pub_fg = nh.advertise<sensor_msgs::PointCloud2>("fgcloud", 1000);

    ros::spin();

    return 0;
}