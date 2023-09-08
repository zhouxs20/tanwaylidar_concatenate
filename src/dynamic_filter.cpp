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
ros::Publisher pub_dyn;
ros::Publisher pub_sta;

PointCloudEX::Ptr lastCloud(new PointCloudEX);
int lastFlag = 0;


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

    // 数值待修改 
    int GndNum = 0;
    int noGndNum = 0;
    float dx = 0.5;
    float dy = 0.5;
    int x_len = 30;
    int y_len = 20;
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
    
    // 筛选出地面点：考虑以点云的形式输出地面点和非地面点，便于后续处理
    for(int pid = 0; pid < inNum; pid++)
    {
        if (GndNum >= 60000)
            break;
        if(idtemp[pid] > 0 && idtemp[pid] < nx*ny)
        {
            imgMeanZ[idtemp[pid]] = float(imgSumZ[idtemp[pid]] / (imgNumZ[idtemp[pid]] + 0.0001));
            //地面点条件：最高点与均值高度差小于阈值；点数大于0；均值高度小于1 
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
    // 释放内存
    free(imgMinZ);
    free(imgMaxZ);
    free(imgSumZ);
    free(imgMeanZ);
    free(imgNumZ);
    free(idtemp);

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

SPcPtr FilterGnd_2(const sensor_msgs::PointCloud2::ConstPtr& Cloud)
{
    // 输入点云类型转到tanway
    PointCloudEX::Ptr inCloud(new PointCloudEX);
    pcl::fromROSMsg(*Cloud, *inCloud);
    // 地面点云和非地面点云
    PointCloudEX::Ptr GndCloud(new PointCloudEX);
    PointCloudEX::Ptr noGndCloud(new PointCloudEX);

    int inNum = inCloud->size();

    // 数值待修改 
    int GndNum = 0;
    int noGndNum = 0;
    float dx = 0.5;
    float dy = 0.5;
    int x_len = 30;
    int y_len = 20;
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
    
    // 筛选出地面点：考虑以点云的形式输出地面点和非地面点，便于后续处理
    for(int pid = 0; pid < inNum; pid++)
    {
        if (GndNum >= 60000)
            break;
        if(idtemp[pid] > 0 && idtemp[pid] < nx*ny)
        {
            imgMeanZ[idtemp[pid]] = float(imgSumZ[idtemp[pid]] / (imgNumZ[idtemp[pid]] + 0.0001));
            //地面点条件：最高点与均值高度差小于阈值；点数大于0；均值高度小于1 
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
    // 释放内存
    free(imgMinZ);
    free(imgMaxZ);
    free(imgSumZ);
    free(imgMeanZ);
    free(imgNumZ);
    free(idtemp);
    
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

/*
    对非背景点进行聚类，两种方案都要用到这个函数
    pLabel：前背景标签，前景=0，背景=1
    seedId：点在点云中的索引
    labelId：簇的编号 maybe
    cloud：是原始点云还是前景点云？可以是前景点云
    kdtree：在当前帧点云里聚类
    fSearchRadius：搜索半径
    thrHeight：
*/
SClusterFeature FindACluster(int *pLabel, int seedId, int labelId, PointCloudEX::Ptr cloud, pcl::KdTreeFLANN<TanwayPCLEXPoint> &kdtree, float fSearchRadius, float thrHeight,std_msgs::Header cheader)
{
    // 初始化种子
    std::vector<int> seeds;
    seeds.push_back(seedId);
    pLabel[seedId]=labelId;
    // int cnum=1;//当前簇里的点数

    SClusterFeature cf;
    cf.pnum=1;
    cf.xmax=-2000;
    cf.xmin=2000;
    cf.ymax=-2000;
    cf.ymin=2000;
    cf.zmax=-2000;
    cf.zmin=2000;
    cf.zmean=0;

    // 区域增长
    while(seeds.size()>0) //实际上是种子点找了下k近邻，然后k近邻再纳入种子点 sy
    {
        int sid=seeds[seeds.size()-1];
        seeds.pop_back();

        TanwayPCLEXPoint searchPoint;
        searchPoint = cloud->points[sid];

        // 特征统计
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

        // if(searchPoint.x<44.8)
        //     kdtree.radiusSearch(searchPoint,fSearchRadius,k_inds,k_dis);
        // else
        //     kdtree.radiusSearch(searchPoint,2*fSearchRadius,k_inds,k_dis);
        kdtree.radiusSearch(searchPoint,fSearchRadius,k_inds,k_dis);
        for(int ind=0;ind<k_inds.size();ind++)
        {
            if(pLabel[k_inds[ind]]==0)
            {
                pLabel[k_inds[ind]]=labelId;
                // cloud->points[k_inds[ind]].intensity=labelId;
                // cnum++;
                cf.pnum++;
                if(cloud->points[k_inds[ind]].z>thrHeight)//地面20cm以下不参与分割
                {
                    seeds.push_back(k_inds[ind]);
                }
            }
        }
    }
    cf.zmean/=(cf.pnum+0.000001);

    // TanwayPCLEXPoint searchPoint;
    // // 动态点云和静态点云
    // PointCloudEX::Ptr dynCloud(new PointCloudEX);
    // if(cf.pnum > 10){
    //     for(int ind=0;ind<k_inds.size();ind++)
    //     searchPoint = cloud->points[seedId];
    //     std::vector<float> k_dis; // 存储近邻点对应距离的平方
    //     std::vector<int> k_inds; // 存储查询近邻点索引
    //     kdtreelast.nearestKSearch(searchPoint, 1, k_inds, k_dis);
    //     if(k_dis[0] > fSearchRadius){
    //         dynCloud->push_back(cloud->points[pid]);
    //     }
    // }
    return cf;
}

void FindCluster(int *pLabel, int seedId, int labelId, PointCloudEX::Ptr cloud, 
pcl::KdTreeFLANN<TanwayPCLEXPoint> &kdtree, pcl::KdTreeFLANN<TanwayPCLEXPoint> &kdtreelast,
float fSearchRadius, float thrHeight,std_msgs::Header cheader)
{
    // 初始化种子
    std::vector<int> seeds;
    seeds.push_back(seedId);
    pLabel[seedId]=labelId;
    int pnum = 1;
    // 聚类点云
    PointCloudEX::Ptr clusterCloud(new PointCloudEX);
    clusterCloud->push_back(cloud->points[seedId]);
    // 区域增长
    while(seeds.size()>0) //实际上是种子点找了下k近邻，然后k近邻再纳入种子点 sy
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
                if(cloud->points[k_inds[ind]].z>thrHeight)//地面20cm以下不参与分割
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
        std::vector<float> k_dis; // 存储近邻点对应距离的平方
        std::vector<int> k_inds; // 存储查询近邻点索引
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

/*
    方案一：通过聚类大小来判断动态点
    pLabel，kdtree，fSearchRadius可以在函数体里初始化
    cloud：前景点云？
*/
int SegObjects(PointCloudEX::Ptr cloud, std_msgs::Header cheader)
{
    int pnum = cloud->points.size();
    int labelId = 10; // object编号从10开始
    int *pLabel=(int*)calloc(pnum, sizeof(int));
    for(int i = 0; i < pnum; i++){
        pLabel[i] = 0;
    }
    //调用pcl的kdtree生成方法
    pcl::KdTreeFLANN<TanwayPCLEXPoint> kdtree;
    kdtree.setInputCloud (cloud);
    float fSearchRadius = 1.2;

    // 动态点云和静态点云
    PointCloudEX::Ptr dynCloud(new PointCloudEX);
    PointCloudEX::Ptr staCloud(new PointCloudEX);

    //遍历每个非背景点，若高度大于0.4则寻找一个簇，并给一个编号(>10)
    for(int pid = 0; pid < pnum; pid++)
    {
        if(pLabel[pid] == 0)
        {
            if(1)//高度阈值:cloud->points[pid].z > 0
            {
                SClusterFeature cf = FindACluster(pLabel,pid,labelId,cloud,kdtree,fSearchRadius,0,cheader);
                int isCar=0;

                // cluster 分类
                float dx=cf.xmax-cf.xmin;
                float dy=cf.ymax-cf.ymin;
                float dz=cf.zmax-cf.zmin;
                // float cx=10000;
                
                // if (cx>cloud->points[pid].x)
                // {
                //     cx=cloud->points[pid].x; // cx是点云x坐标的最小值
                // }

                // if((dx> 10)||(dy>10)||((dx>6)&&(dy>6)))// 太大
                // {
                //     isBg=2;
                // }
                // else if(((dx>6)||(dy>6))&&(cf.zmean < 2))//长而过低
                // {
                //     isBg = 3;//1;//2;
                // }
                // else if(((dx<1.5)&&(dy<1.5)))//小而过高
                // {
                //     isBg = 4;
                // }
                // else if(cf.pnum<5 || (cf.pnum<10 && cx<50)) //点太少
                // {
                //     isBg=5;
                // }
                // else if((cf.zmean>3)||(cf.zmean<0.3))//太高或太低
                // {
                //     isBg = 6;//1;
                // }
                // if((dx < 2) && (dy > 1) && (dy < 2) && (dz > 1) && (dz < 2)){ // 小车平行方向直行
                //     isCar = 1;
                // }
                // else if((dx < 2) && (dy > 1.5) && (dy < 2.5) && (dz > 1.5) && (dz < 3)){ // 大车平行方向直行
                //     isCar = 1;
                // }
                // else if((dx < 2) && (dy > 3) && (dy < 5) && (dz > 1) && (dz < 2)){ // 小车垂直方向直行
                //     isCar = 1;
                // }
                // else if((dx < 2) && (dy > 4) && (dy < 10) && (dz > 1.5) && (dz < 3)){ // 大车垂直方向直行
                //     isCar = 1;
                // }
                //  std::cout << "动态点数：" << cf.pnum << std::endl;

                // 筛选动态点条件：cf.pnum >= 60 && (dy >=1 && dx >= 1) && cf.zmax < 1.5
                if(cf.pnum > 5 && cf.pnum <= 200 && (dx < 2) && (dy > 1) && (dy < 2.5)){
                    isCar = 1;
                }
                if(cf.pnum > 5 && cf.pnum <= 200 && (dy < 3) && (dx > 1) && (dx < 4)){
                    isCar = 1;
                }

                if(isCar>0) // 归入动态点
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
    // 发布点云
    std::cout << dynCloud->size() << std::endl;
    std::cout << staCloud->size() << std::endl;
    
    sensor_msgs::PointCloud2 msg_dyn;
    pcl::toROSMsg(*dynCloud, msg_dyn);
    msg_dyn.header = cheader; //header怎么传？
    pub_dyn.publish(msg_dyn);

    sensor_msgs::PointCloud2 msg_sta;
    pcl::toROSMsg(*staCloud, msg_sta);
    msg_sta.header = cheader;
    pub_sta.publish(msg_sta);
    return labelId-10;//返回前景类个数
}

/*
    方案二：通过前后帧对比来判断
*/
void SegDynamic(PointCloudEX::Ptr lastCloud, PointCloudEX::Ptr fgCloud, std_msgs::Header cheader){
    int fgNum = fgCloud->points.size(); // 当前前景点云的点数
    // std::cout << fgNum << std::endl;
    // 动态点云和静态点云
    PointCloudEX::Ptr dynCloud(new PointCloudEX);
    PointCloudEX::Ptr staCloud(new PointCloudEX);

    // object编号从10开始
    int labelId = 10; 
    int *pLabel=(int*)calloc(fgNum, sizeof(int));
    for(int i = 0; i < fgNum; i++){
        pLabel[i] = 0;
    }

    //调用pcl的kdtree生成方法
    pcl::KdTreeFLANN<TanwayPCLEXPoint> kdtree;
    kdtree.setInputCloud(fgCloud);
    pcl::KdTreeFLANN<TanwayPCLEXPoint> kdtreelast;
    kdtreelast.setInputCloud(lastCloud);
    int searchNum = 1;
    float fSearchRadius = 1; // 调整
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
    // std::cout << "ok" << std::endl;
    // 发布点云
    std::cout << dynCloud->size() << std::endl;
    std::cout << staCloud->size() << std::endl;
    
    sensor_msgs::PointCloud2 msg_dyn;
    pcl::toROSMsg(*dynCloud, msg_dyn);
    msg_dyn.header = cheader; //header怎么传？
    pub_dyn.publish(msg_dyn);

    sensor_msgs::PointCloud2 msg_sta;
    pcl::toROSMsg(*staCloud, msg_sta);
    msg_sta.header = cheader;
    pub_sta.publish(msg_sta);
}

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
    // if(lastFlag == 0){
    //     *lastCloud = *inCloud;
    // }
    
    // 背景点云和前景点云
    PointCloudEX::Ptr bgCloud(new PointCloudEX);
    PointCloudEX::Ptr fgCloud(new PointCloudEX);

    // 初始化种子点，选取高度在4-6m之间的所有点，将索引存入seeds
    std::vector<int> seeds;
    for (int pid = 0; pid < inNum; pid++)
    {
        if(inCloud->points[pid].z > 3)
        {
            pLabel[pid] = 1; // 背景
            if (inCloud->points[pid].z < 5)
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
        kdtree.radiusSearch(sid, fSearchRadius, k_inds, k_dis);
        // if (inCloud->points[sid].x < 44.8) // 限制了x的范围，对于近点和远点采用不同的半径
        //     kdtree.radiusSearch(sid, fSearchRadius, k_inds, k_dis);
        // else
        //     kdtree.radiusSearch(sid, 1.5*fSearchRadius, k_inds, k_dis);

        for(int ind=0;ind<k_inds.size();ind++)
        {
            if(pLabel[k_inds[ind]]==0) // 前景=0
            {
                pLabel[k_inds[ind]]=1; // 归到背景=1
                seeds.push_back(k_inds[ind]);
                // if(inCloud->points[k_inds[ind]].z>0)//（防止增长到了近地面）地面20cm以下不参与背景分割，以防止错误的地面点导致过度分割
                // {
                //     seeds.push_back(k_inds[ind]);
                // }
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
    // std::cout << bgCloud->size() << std::endl;
    // std::cout << fgCloud->size() << std::endl;

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
    //     SegDynamic(lastCloud, fgCloud, Cloud->header);
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