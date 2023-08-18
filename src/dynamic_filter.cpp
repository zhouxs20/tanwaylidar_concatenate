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

/*
    对非背景点进行聚类
    pLabel：前背景标签，前景=0，背景=1
    seedId：点在点云中的索引
    labelId：簇的编号 maybe
    cloud：是原始点云还是前景点云？可以是前景点云
    kdtree：在当前帧点云里聚类
    fSearchRadius：搜索半径
    thrHeight：
*/
SClusterFeature FindACluster(int *pLabel, int seedId, int labelId, PointCloudEX::Ptr cloud, pcl::KdTreeFLANN<TanwayPCLEXPoint> &kdtree, float fSearchRadius, float thrHeight)
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
        // searchPoint.x=cloud->points[sid].x;
        // searchPoint.y=cloud->points[sid].y;
        // searchPoint.z=cloud->points[sid].z;

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

        if(searchPoint.x<44.8)
            kdtree.radiusSearch(searchPoint,fSearchRadius,k_inds,k_dis);
        else
            kdtree.radiusSearch(searchPoint,2*fSearchRadius,k_inds,k_dis);

        for(int ind=0;ind<k_inds.size();ind++)
        {
            if(pLabel[k_inds[ind]]==0)
            {
                pLabel[k_inds[ind]]=labelId;
                // cnum++;
                cf.pnum++;
                if(cloud->points[k_inds[ind]].z>thrHeight)//地面60cm以下不参与分割
                {
                    seeds.push_back(k_inds[ind]);
                }
            }
        }
    }
    cf.zmean/=(cf.pnum+0.000001);
    return cf;
}

/*
pLabel，kdtree，fSearchRadius可以在函数体里初始化
cloud：前景点云？
*/
int SegObjects(PointCloudEX::Ptr cloud, std_msgs::Header cheader)
{
    int pnum = cloud->points.size();
    int labelId = 10; // object编号从10开始 ?
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
            if(cloud->points[pid].z > 0.4)//高度阈值
            {
                SClusterFeature cf = FindACluster(pLabel,pid,labelId,cloud,kdtree,fSearchRadius,0.2);
                int isBg=0;

                // cluster 分类
                float dx=cf.xmax-cf.xmin;
                float dy=cf.ymax-cf.ymin;
                float dz=cf.zmax-cf.zmin;
                float cx=10000;
                for(int ii=0;ii<pnum;ii++)
                {
                    if (cx>cloud->points[pid].x)
                    {
                        cx=cloud->points[pid].x; // cx是点云x坐标的最小值
                    }
                }

                if((dx>10)||(dy>10)||((dx>6)&&(dy>6)))// 太大
                {
                    isBg=2;
                }
                else if(((dx>6)||(dy>6))&&(cf.zmean < 2))//长而过低
                {
                    isBg = 3;//1;//2;
                }
                else if(((dx<1.5)&&(dy<1.5)))//小而过高
                {
                    isBg = 4;
                }
                else if(cf.pnum<5 || (cf.pnum<10 && cx<50)) //点太少
                {
                    isBg=5;
                }
                else if((cf.zmean>3)||(cf.zmean<0.3))//太高或太低
                {
                    isBg = 6;//1;
                }

                if(isBg>0) // 归入背景
                {
                    for(int ii=0;ii<pnum;ii++)
                    {
                        if(pLabel[ii]==labelId)
                        {
                            pLabel[ii]=isBg;
                        }
                    }
                }
                else // 前景
		        {
                    labelId++;
                }
            }
        }
    }
    for(int pid = 0; pid < pnum; pid++){
        if(pLabel[pid] > 0){
            staCloud->push_back((*cloud)[pid]);
        }
        else if(pLabel[pid] == 0){
            dynCloud->push_back((*cloud)[pid]);
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

void SegDynamic(PointCloudEX::Ptr lastCloud, PointCloudEX::Ptr fgCloud, std_msgs::Header cheader){
    int fgNum = fgCloud->points.size(); // 当前前景点云的点数
    // std::cout << fgNum << std::endl;
    // 动态点云和静态点云
    PointCloudEX::Ptr dynCloud(new PointCloudEX);
    PointCloudEX::Ptr staCloud(new PointCloudEX);

    //调用pcl的kdtree生成方法
    pcl::KdTreeFLANN<TanwayPCLEXPoint> kdtree;
    kdtree.setInputCloud(lastCloud);
    int searchNum = 1;
    float fSearchRadius = 0.2; // 调整
    TanwayPCLEXPoint searchPoint;
    

    for (int pid = 0; pid < fgNum; pid++){
        searchPoint = fgCloud->points[pid];
        // searchPoint.x = fgCloud->points[pid].x;
        // searchPoint.y = fgCloud->points[pid].y;
        // searchPoint.z = fgCloud->points[pid].z;
        std::vector<float> k_dis; // 存储近邻点对应距离的平方
        std::vector<int> k_inds; // 存储查询近邻点索引
        // std::cout << "kdtree" << std::endl;
        kdtree.nearestKSearch(searchPoint, searchNum, k_inds, k_dis); // pid位置放什么参数？
        // std::cout << k_dis[0] << std::endl;
        if(k_dis[0] > fSearchRadius){
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
    int fgNum = SegObjects(fgCloud, Cloud->header);
    std::cout << "动态点数：" << fgNum << std::endl;

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

    ros::Subscriber sub = nh.subscribe(topic_pcl, 10, SegBG);
    pub_g = nh.advertise<sensor_msgs::PointCloud2>("gndcloud", 10);
    pub_ng = nh.advertise<sensor_msgs::PointCloud2>("nogndcloud", 10);
    pub_bg = nh.advertise<sensor_msgs::PointCloud2>("bgcloud", 1000);
    pub_fg = nh.advertise<sensor_msgs::PointCloud2>("fgcloud", 1000);
    pub_dyn = nh.advertise<sensor_msgs::PointCloud2>("dyncloud", 1000);
    pub_sta = nh.advertise<sensor_msgs::PointCloud2>("stacloud", 1000);

    ros::spin();

    return 0;
}