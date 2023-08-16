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

#include <vector>

using namespace std;

int FilterGndForPos_cor(float* outPoints,float*inPoints,int inNum)
{
    // 数值待修改
    int outNum = 0;
    float dx = 2;
    float dy = 2;
    int x_len = 20;
    int y_len = 10;
    int nx = 2 * x_len / dx; //80，乘2是因为范围在(-x_len,x_len)之间
    int ny = 2 * y_len / dy; //10
    float offx = -20, offy = -10;
    float THR = 0.4;
    
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
        if((inPoints[pid*4] > -x_len) && (inPoints[pid*4] < x_len) && (inPoints[pid*4+1] > -y_len) && (inPoints[pid*4+1] < y_len)) //范围在(-x_len,x_len)之间
        {
            int idx = (inPoints[pid*4] - offx) / dx; // 点云坐标的存储方式可能需要修改
            int idy = (inPoints[pid*4+1] - offy) / dy;
            idtemp[pid] = idx + idy * nx; //获得该点的索引
            if (idtemp[pid] >= nx * ny)
                continue;
            imgSumZ[idx+idy*nx] += inPoints[pid*4+2];
            imgNumZ[idx+idy*nx] += 1;
            if(inPoints[pid*4+2] < imgMinZ[idx+idy*nx])
            {
                imgMinZ[idx+idy*nx] = inPoints[pid*4+2];
            }
            if(inPoints[pid*4+2] > imgMaxZ[idx+idy*nx]){
                imgMaxZ[idx+idy*nx] = inPoints[pid*4+2];
            }
        }
    }
    // 筛选出地面点：考虑以点云的形式输出地面点和非地面点，便于后续处理
    for(int pid = 0; pid < inNum; pid++)
    {
        if (outNum >= 60000)
            break;
        if(idtemp[pid] > 0 && idtemp[pid] < nx*ny)
        {
            imgMeanZ[idtemp[pid]] = float(imgSumZ[idtemp[pid]] / (imgNumZ[idtemp[pid]] + 0.0001));
            //地面点条件：最高点与均值高度差小于阈值；点数大于3；均值高度小于1 
            if((imgMaxZ[idtemp[pid]] - imgMeanZ[idtemp[pid]]) < THR && imgNumZ[idtemp[pid]] > 3 && imgMeanZ[idtemp[pid]] < 2)
            {
                outPoints[outNum*4]=inPoints[pid*4];
                outPoints[outNum*4+1]=inPoints[pid*4+1];
                outPoints[outNum*4+2]=inPoints[pid*4+2];
                outPoints[outNum*4+3]=inPoints[pid*4+3];
                outNum++;
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
    return outNum;
}

int SegBG(int *pLabel, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, float fSearchRadius)
{
    //从较高的一些点开始从上往下增长，这样能找到一些类似树木、建筑物这样的高大背景
    // clock_t t0, t1, tsum = 0;
    int inNum = cloud->points.size(); // 输入点云的点数
    pcl::PointXYZ searchPoint;

    // 初始化种子点，选取高度在4-6m之间的所有点，将索引存入seeds
    std::vector<int> seeds;
    for (int pid = 0; pid < inNum; pid++)
    {
        if(cloud->points[pid].z > 4)
        {
            pLabel[pid] = 1; // 背景
            if (cloud->points[pid].z < 6)
            {
                seeds.push_back(pid);
            }
        }
        else
        {
            pLabel[pid]=0; // 暂时归进前景
        }

    }

    // 区域增长
    while(seeds.size() > 0)
    {
        int sid = seeds[seeds.size()-1]; // 从后往前依次增长
        seeds.pop_back();

        std::vector<float> k_dis; // 存储近邻点对应距离的平方
        std::vector<int> k_inds; // 存储查询近邻点索引
        // t0 = clock();

        // 查询point半径为radius邻域球内的点，搜索结果默认是按照距离point点的距离从近到远排序
        if (cloud->points[sid].x < 44.8) // 限制了x的范围，对于近点和远点采用不同的半径
            kdtree.radiusSearch(sid, fSearchRadius, k_inds, k_dis);
        else
            kdtree.radiusSearch(sid, 1.5*fSearchRadius, k_inds, k_dis);

        for(int ii=0;ii<k_inds.size();ii++)
        {
            if(pLabel[k_inds[ii]]==0) // 前景=0
            {
                pLabel[k_inds[ii]]=1; // 归到背景=1
                if(cloud->points[k_inds[ii]].z>0.2)//（防止增长到了近地面）地面60cm以下不参与背景分割，以防止错误的地面点导致过度分割
                {
                    seeds.push_back(k_inds[ii]);
                }
            }
        }

    }
    return 0;

}