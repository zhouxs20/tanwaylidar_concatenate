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

# define pcl_isfinite(x) std::isfinite(x)

std::string topic_pcl = "/tanwaylidar_undistort";
std::string topic_imu = "/imu";
std::string topic_gps = "/gps";

ros::Publisher pub_pcl;
ros::Publisher pub;
ros::Publisher pub_bg;

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
std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
double t_0[3] = {0,0,0};
int cnt = 0;
const int frame_num = 5;
double oxts[frame_num][6] = {};
double scale = 0;
double er = 6378137.0000;
float min_dist_ratio_ = 0.05;

// 将imu四元数转换为欧拉角
void imu_to_rpy(double x, double y, double z, double w, double* roll, double* pitch, double* yaw)
{   
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3(q).getRPY(*roll, *pitch, *yaw);
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

//矩阵相乘函数（列1）
void Multiply1(float a[], float b[], float c[][1], int n, int k, int m){
    for (int i = 0; i < n; i++){
        for (int j = 0; j < m; j++){
            c[i][j] = 0;
            for (int v = 0; v < k; v++){
                c[i][j] += a[i*k+v] * b[v*m+j];
            }
        }
    }
}

//矩阵相乘函数（列3）
void Multiply3(float a[], float b[], float c[][3], int n, int k, int m){
    for (int i = 0; i < n; i++){
        for (int j = 0; j < m; j++){
            c[i][j] = 0;
            for (int v = 0; v < k; v++){
                c[i][j] += a[i*k+v] * b[v*m+j];
            }
        }
    }
}

float **poses_from_oxts(Oxts oxts){
    float **p = new float *[4]; // 创建动态二维数组p
    for(int i = 0; i < 4; i++){
        p[i] = new float[4];
    }
    
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
    float c[3][3] = {0};
    float R[3][3] = {0};
    Multiply3(*Ry, *Rx, c, 3, 3, 3);

    int z = 0;
    float Rz1[9],c1[9];
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            Rz1[z] = Rz[i][j];
            c1[z] = c[i][j];
            z++;
        }
    }
    Multiply3(Rz1, c1, R, 3, 3, 3);

    for(int i = 0; i<3; i++){
        t[i] = t[i] - t_0[i];
    }
    // 将旋转矩阵R和平移矩阵t拼到一起
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

void remove_bg(PointCloudEX::Ptr& cloud_bg, PointCloudEX::Ptr& curcloud){
    PointCloudEX::Ptr cloud(new PointCloudEX);
    PointCloudEX::Ptr cloud_fg(new PointCloudEX);
    *cloud = *curcloud;
    TanwayPCLEXPoint invalid;
    invalid.x = invalid.y = invalid.z = std::numeric_limits<float>::quiet_NaN();
    int pointNum = min({cloud->size(), cloud_bg->size()});
    int invalidNum = 0;
    for (size_t i = 0; i < pointNum; i++)
    {
        const TanwayPCLEXPoint &p_in = cloud->points[i];
        const TanwayPCLEXPoint &p_bg = cloud_bg->points[i];

        // if input point invalid -> output invalid
        if (!(pcl_isfinite(p_in.x) && pcl_isfinite(p_in.y) && pcl_isfinite(p_in.z))){
            continue;
        }
        if (!(pcl_isfinite(p_bg.x) && pcl_isfinite(p_bg.y) && pcl_isfinite(p_bg.z))){
            continue;
        }

        // compare ratio between background and input point to threshold
        float dist_in = p_in.getVector3fMap().norm();
        float dist_bg = p_bg.getVector3fMap().norm();
        float diff = dist_bg - dist_in;

        if (diff < 0){
            // point is further away than previous background
            (*cloud_bg)[i] = p_in;
            cloud_fg->push_back((*cloud)[i]);
            (*cloud)[i] = invalid;
            invalidNum++;
        }
        else if (diff / dist_bg < min_dist_ratio_){
            // point is too close to background
            cloud_fg->push_back((*cloud)[i]);
            (*cloud)[i] = invalid;
            invalidNum++;
        }
        std::cout<<invalidNum<<std::endl;
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    // msg.header = cloud->header;
    pub.publish(msg);

    sensor_msgs::PointCloud2 msg_bg;
    pcl::toROSMsg(*cloud_fg, msg_bg);
    // msg_bg.header = cloud_bg->header;
    pub_bg.publish(msg_bg);
}

void create_pcd(){
    sensor_msgs::PointCloud2 pcl_new_msg;  //等待发送的点云消息
    PointCloudEX mid_pcl;   //建立了一个pcl的点云，作为中间过程
    
    int num = 0;
    sensor_msgs::PointCloud2::ConstPtr pcloud(new sensor_msgs::PointCloud2);
    PointCloudEX::Ptr cloudadd(new PointCloudEX);
    PointCloudEX::Ptr cloud(new PointCloudEX);

    PointCloudEX::Ptr lastcloud(new PointCloudEX);
    PointCloudEX::Ptr currentcloud(new PointCloudEX);
    PointCloudEX::Ptr bgcloud(new PointCloudEX);

    for(int i = 0; i < frame_num; i++){
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
        double yaw;
        yaw = oxts_buffer[frame_num-1].yaw;

        // 以下为pcd点云处理部分
        int pc_row;
        double x,y;
        if(num == 0){
            pcloud = lidar_buffer[num];
            pcl::fromROSMsg(*pcloud, *cloudadd);
            pc_row = cloudadd->points.size();
            // std::cout<<"原始点云"<<cloudadd->points.size()<<std::endl;
            float pc_array[4];
            for(int r = 0; r < pc_row; r++){
                pc_array[0] = cloudadd->points[r].x;
                pc_array[1] = cloudadd->points[r].y;
                pc_array[2] = cloudadd->points[r].z;
                pc_array[3] = 1.0;
                float pc[4][1] = {0};
                Multiply1(pose1, pc_array, pc, 4, 4, 1);
                // cloudadd->points[r].x = pc[0][0];
                // cloudadd->points[r].y = pc[1][0];
                x = pc[0][0];
                y = pc[1][0];
                cloudadd->points[r].z = pc[2][0];
                cloudadd->points[r].x = x * cos(yaw) + y * sin(yaw);
                cloudadd->points[r].y = -x * sin(yaw) + y * cos(yaw);
            }
        }
        else if(num > 0){
            pcloud = lidar_buffer[num];
            pcl::fromROSMsg(*pcloud, *cloud);
            // cloud = pcl_buffer[num];
            pc_row = cloud->points.size();
            // std::cout<<"原始点云"<<cloud->points.size()<<std::endl;
            float pc_array[4];
            for(int r = 0; r < pc_row; r++){
                pc_array[0] = cloud->points[r].x;
                pc_array[1] = cloud->points[r].y;
                pc_array[2] = cloud->points[r].z;
                pc_array[3] = 1.0;
                float pc[4][1] = {0};
                Multiply1(pose1, pc_array, pc, 4, 4, 1);
                // cloud->points[r].x = pc[0][0];
                // cloud->points[r].y = pc[1][0];
                
                x = pc[0][0];
                y = pc[1][0];
                cloud->points[r].z = pc[2][0];
                cloud->points[r].x = x * cos(yaw) + y * sin(yaw);
                cloud->points[r].y = -x * sin(yaw) + y * cos(yaw);
            }
            if(num == frame_num - 2){
                *lastcloud = *cloud;
            }
            else if(num == frame_num - 1){
                *currentcloud = * cloud;
                remove_bg(lastcloud, currentcloud);
            }
            *cloudadd = *cloudadd + *cloud;
            cloud->clear();
        }
        
        num += 1;
    }
    mid_pcl = *cloudadd;
    // std::cout<<"拼接点云"<<cloudadd->points.size()<<std::endl;
    pcl::toROSMsg(mid_pcl, pcl_new_msg);  //将点云转化为消息才能发布
    pub_pcl.publish(pcl_new_msg); //发布调整之后的点云数据，主题为/new_cloud
    // pcl::io::savePCDFileBinary("mytest_pcd.pcd", *cloudadd);
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& pclMsg, const sensor_msgs::NavSatFix::ConstPtr& gpsMsg, const sensor_msgs::Imu::ConstPtr& imuMsg){
    cnt++;
    std::cout << cnt << std::endl; // 点云数
    Oxts oxt;
    
    
    if(lidar_buffer.size()==frame_num){
        lidar_buffer.pop_front();
        // std::cout<<"点云队列存储已满"<<std::endl;
    }
    lidar_buffer.push_back(pclMsg);
    // gps数据
    sensor_msgs::NavSatFix::Ptr gpsmsg(new sensor_msgs::NavSatFix(*gpsMsg));
    // double lon, lat, alt;
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
    if(oxts_buffer.size() < frame_num){
        oxts_buffer.push_back(oxt);
    }
    else if(oxts_buffer.size() == frame_num){
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
    pub = nh.advertise<sensor_msgs::PointCloud2>("foreground", 10);
    pub_bg = nh.advertise<sensor_msgs::PointCloud2>("background", 10);

    ros::spin();

    return 0;
}