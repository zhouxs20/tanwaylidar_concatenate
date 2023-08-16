#include <iostream>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <std_msgs/String.h>
#include <rosbag/view.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include <sensor_msgs/PointCloud2.h> 
#include "tf/transform_datatypes.h"//转换函数头文件
#include <geometry_msgs/Point32.h>//geometry_msgs消息数据类型
#include <tf/tf.h>
#include <dirent.h>
#include <cmath>
#include <queue>
#include <csignal>
#include <condition_variable>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h> //use these to convert between PCL and ROS datatypes

using namespace std;
using namespace message_filters;

std::string topic_pcl = "/tanwaylidar_pointcloud";
std::string topic_imu = "/imu";
std::string topic_gps = "/gps";

ros::Publisher pub_pcl;

struct TanwayPCLEXPoint
{
    PCL_ADD_POINT4D;
	float intensity;
  	int channel;
	float angle;
	int echo;
	int block;				/*For duetto*/
  	unsigned int t_sec;     /* The value represents seconds since 1900-01-01 00:00:00 (the UNIX epoch).*/ 
	unsigned int t_usec;    /* remaining microseconds */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(TanwayPCLEXPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (int, channel, channel)
                                  (float, angle, angle)	
                                  (int, echo, echo)
								  (int, block, block)
								  (unsigned int, t_sec, t_sec)
								  (unsigned int, t_usec, t_usec)
                                 )

typedef pcl::PointCloud<TanwayPCLEXPoint> PointCloudEX;

// 用于存放每一帧的gps和imu信息
struct Oxts
{
    double lon;
    double lat;
    double alt;
    double roll;
    double pitch;
    double yaw;
};

std::deque<Oxts> oxts_buffer;
std::deque<PointCloudEX::Ptr> pcl_buffer;
double t_0[3] = {0,0,0};
int cnt = 0;
double oxts[5][6] = {};
double scale = 0;
double er = 6378137.0000;

// 将imu四元数转换为欧拉角
void imu_to_rpy(double x, double y, double z, double w, double* roll, double* pitch, double* yaw)
{   
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3(q).getRPY(*roll, *pitch, *yaw);
    // *roll = *roll * 180 / M_PI;
    // *pitch = *pitch * 180 / M_PI;
    // *yaw = *yaw * 180 / M_PI;
}

void GetFileNames(string path, vector<string> &filenames, string form)
{
    DIR *pDir;
    struct dirent *ptr;
    string filename;
    if (!(pDir = opendir(path.c_str())))
        return;
    while ((ptr = readdir(pDir)) != 0)
    {
        // 跳过.和..文件
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
            continue;
        filename = ptr->d_name;
        filenames.push_back(filename);
    }
    closedir(pDir);
    //按字符串的方式进行  
    sort(filenames.begin(), filenames.end());
}

void rotx(double t, float matx[][3]){
    float c, s;
    c = cos(t);
    s = sin(t);
    matx[1][1] = c;
    matx[1][2] = -s;
    matx[2][1] = s;
    matx[2][2] = c;
}

void roty(double t, float matx[][3]){
    float c, s;
    c = cos(t);
    s = sin(t);
    matx[0][0] = c;
    matx[0][2] = s;
    matx[2][0] = -s;
    matx[2][2] = c;
}

void rotz(double t, float matx[][3]){
    float c, s;
    c = cos(t);
    s = sin(t);
    matx[0][0] = c;
    matx[0][1] = -s;
    matx[1][0] = s;
    matx[1][1] = c;
}

//矩阵相乘函数
float **Multiply(float a[], float b[], int n, int k, int m){
    float **c = new float *[n]; // 创建动态二维数组c
    for(int i = 0; i < n; i++){
        c[i] = new float[m];
    }
    for (int i = 0; i < n; i++){
        for (int j = 0; j < m; j++){
            c[i][j] = 0;
            for (int v = 0; v < k; v++){
                c[i][j] += a[i*k+v] * b[v*m+j];
            }
        }
    }
    return c;
}

//矩阵拼接
float **transform_from_rot_trans(float R[], float t[]){
    float **p = new float *[4]; // 创建动态二维数组p
    for(int i = 0; i < 4; i++){
        p[i] = new float[4];
    }
    for(int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            p[i][j] = R[i*4+j];
        }
    }
    for(int i = 0; i < 3; i++){
        p[i][3] = t[i];
    }
    float a[4] = {0, 0, 0, 1};
    for(int j = 0; j < 4; j++){
        p[3][j] = a[j];
    }
    return p;
}

float **poses_from_oxts(Oxts oxts){
    float **p = new float *[4]; // 创建动态二维数组p
    for(int i = 0; i < 4; i++){
        p[i] = new float[4];
    }
    
    // float scale = cos(oxts.lat * M_PI / 180);
    double tx, ty, tz;
    tx = scale * oxts.lon * M_PI * er / 180;
    ty = scale * er * log(tan((90 + oxts.lat) * M_PI / 360));
    tz = oxts.alt;
    double t[3] = {tx, ty, tz};
    
    float Rx[3][3] = {{1, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    float Ry[3][3] = {{0, 0, 0}, {0, 1, 0}, {0, 0, 0}};
    float Rz[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 1}};
    rotx(oxts.roll, Rx);
    roty(oxts.pitch, Ry);
    rotz(oxts.yaw, Rz);
    
    float **c, **R;
    c = Multiply(*Ry, *Rx, 3, 3, 3);
    int z = 0;
    float Rz1[9],c1[9];
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            Rz1[z] = Rz[i][j];
            c1[z] = c[i][j];
            z++;
        }
    }
    R = Multiply(Rz1, c1, 3, 3, 3);
    for(int i = 0; i<3; i++){
        t[i] = t[i] - t_0[i];
    }
    // 以下相当于p = transform_from_rot_trans(*R, t);
    for(int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            p[i][j] = R[i][j];
        }
    }
    for(int i = 0; i < 3; i++){
        p[i][3] = t[i];
    }
    float a[4] = {0, 0, 0, 1};
    for(int j = 0; j < 4; j++){
        p[3][j] = a[j];
    }
    return p;
}

void create_pcd(){
    sensor_msgs::PointCloud2 pcl_new_msg;  //等待发送的点云消息
    PointCloudEX mid_pcl;   //建立了一个pcl的点云，作为中间过程
    int frame = 5;
    int num = 0;
    PointCloudEX::Ptr cloudadd(new PointCloudEX);
    PointCloudEX::Ptr cloud(new PointCloudEX);
    for(int i = 0; i < frame; i++){
        float **pose;
        pose = poses_from_oxts(oxts_buffer[i]);
        // 输出位姿矩阵
        std::cout << i << std::endl;
        for(int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++){
                std::cout << pose[i][j] << ",";
            }
            std::cout << endl;
        }
        int z = 0;
        float pose1[16];
        for(int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++){
                pose1[z] = pose[i][j];
                z++;
            }
        }
        // 以下为pcd点云处理部分
        int pc_row;
        if(num == 0){
            cloudadd = pcl_buffer[num];
            pc_row = cloudadd->points.size();
            float pc_array[4];
            for(int r = 0; r < pc_row; r++){
                pc_array[0] = cloudadd->points[r].x;
                pc_array[1] = cloudadd->points[r].y;
                pc_array[2] = cloudadd->points[r].z;
                pc_array[3] = 1.0;
                float **pc;
                pc = Multiply(pose1, pc_array, 4, 4, 1);
                cloudadd->points[r].x = pc[0][0];
                cloudadd->points[r].y = pc[1][0];
                cloudadd->points[r].z = pc[2][0];
            }
        }
        else if(num > 0){
            cloud = pcl_buffer[num];
            pc_row = cloud->points.size();
            float pc_array[4];
            for(int r = 0; r < pc_row; r++){
                pc_array[0] = cloud->points[r].x;
                pc_array[1] = cloud->points[r].y;
                pc_array[2] = cloud->points[r].z;
                pc_array[3] = 1.0;
                float **pc;
                pc = Multiply(pose1, pc_array, 4, 4, 1);
                cloud->points[r].x = pc[0][0];
                cloud->points[r].y = pc[1][0];
                cloud->points[r].z = pc[2][0];
            }
            *cloudadd = *cloudadd + *cloud;
            cloud->clear();
        }
        num += 1;
    }
    mid_pcl = *cloudadd;
    pcl::toROSMsg(mid_pcl, pcl_new_msg);  //将点云转化为消息才能发布
    pub_pcl.publish(pcl_new_msg); //发布调整之后的点云数据，主题为/new_cloud
    // pcl::io::savePCDFileBinary("mytest_pcd.pcd", *cloudadd);
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& pclMsg, const sensor_msgs::NavSatFix::ConstPtr& gpsMsg, const sensor_msgs::Imu::ConstPtr& imuMsg){
    cnt++;
    std::cout << cnt << std::endl; // 点云数
    Oxts oxt;
    int frame = 5;
    // 点云数据
    PointCloudEX::Ptr pclPtr(new PointCloudEX);
    pcl::fromROSMsg(*pclMsg, *pclPtr);
    if(pcl_buffer.size()==5){
        pcl_buffer.pop_front();
    }
    pcl_buffer.push_back(pclPtr);
    // gps数据
    sensor_msgs::NavSatFix::Ptr gpsmsg(new sensor_msgs::NavSatFix(*gpsMsg));
    double lon, lat, alt;
    oxt.lon = gpsmsg->longitude;
    oxt.lat = gpsmsg->latitude;
    oxt.alt = gpsmsg->altitude;
    // imu数据
    sensor_msgs::Imu::Ptr imumsg(new sensor_msgs::Imu(*imuMsg));
    double ortx, orty, ortz, ortw;
    ortx = imumsg->orientation.x;
    orty = imumsg->orientation.y;
    ortz = imumsg->orientation.z;
    ortw = imumsg->orientation.w;
    double roll=0, pitch=0, yaw=0;
    imu_to_rpy(ortx, orty, ortz, ortw, &roll, &pitch, &yaw);
    oxt.roll = roll;
    oxt.pitch = pitch;
    oxt.yaw = yaw;
    // 将数据存进队列
    if(oxts_buffer.size() < 5){
        oxts_buffer.push_back(oxt);
    }
    else if(oxts_buffer.size() == 5){
        oxts_buffer.pop_front();
        oxts_buffer.push_back(oxt);
        // 计算t0
        scale = cos(oxt.lat * M_PI / 180);
        t_0[0] = scale * oxt.lon * M_PI * er / 180;
        t_0[1] = scale * er * log(tan((90 + oxt.lat) * M_PI / 360));
        t_0[2] = oxt.alt;
        create_pcd();
    }
}

int main(int argc, char** argv){
    // 初始化ROS节点
    ros::init(argc, argv, "play_bag_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pcl(nh, topic_pcl, 10);
    message_filters::Subscriber<sensor_msgs::NavSatFix> sub_gps(nh, topic_gps, 10);
    message_filters::Subscriber<sensor_msgs::Imu> sub_imu(nh, topic_imu, 10);

    // 将3个话题的数据进行同步，ApproximateTime Policy方法根据输入消息的时间戳进行近似匹配，不要求消息时间完全相同
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::NavSatFix, sensor_msgs::Imu> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_pcl, sub_gps, sub_imu);
    // 指定一个回调函数，就可以实现3个话题数据的同步获取    
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    
    //建立一个发布器，主题是/new_cloud，方便之后发布调整后的点云
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/new_cloud", 1000);  
    
    ros::spin();

    return 0;
}