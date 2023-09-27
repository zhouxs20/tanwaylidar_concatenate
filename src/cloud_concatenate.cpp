#include <iostream>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <std_msgs/String.h>
#include <rosbag/view.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include <sensor_msgs/PointCloud2.h> 
#include "tf/transform_datatypes.h"
#include <geometry_msgs/Point32.h>
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

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/impl/flann_search.hpp>


using namespace std;
using namespace message_filters;


std::string topic_pcl = "/tanwaylidar_undistort";
std::string topic_imu = "/imu";
std::string topic_gps = "/gps";

ros::Publisher pub_pcl;
ros::Publisher pub;
ros::Publisher pub_bg;
ros::Publisher pub_dyn;
ros::Publisher pub_sta;

struct TanwayPCLEXPoint{
    PCL_ADD_POINT4D;
	float intensity;
  	int channel;
	float angle;
	int echo;
	int block;				/* For duetto */
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


/** 
 * @brief Store GPS and IMU information for each frame.
 */
struct Oxts{
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
const int frame_num = 5; // number of frames spliced
double oxts[frame_num][6] = {};
double scale = 0;
double er = 6378137.0000;
float min_dist_ratio_ = 0.02;


/** 
 * @brief Convert imu quaternions to Euler angles.
 * @param x,y,z,w           imu quaternions
 * @param roll,pitch,yaw    Euler angles
 */
void imu_to_rpy(double x, double y, double z, double w, double* roll, double* pitch, double* yaw)
{   
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3(q).getRPY(*roll, *pitch, *yaw);
}


/** 
 * @brief Calculate rotation matrix in x-axis
 * @param t       roll
 * @param matx    a 3x3 rotation matrix
 */
void rotx(double t, float matx[][3]){
    float c, s;
    c = cos(t);
    s = sin(t);
    matx[1][1] = c;
    matx[1][2] = -s;
    matx[2][1] = s;
    matx[2][2] = c;
}


/** 
 * @brief Calculate rotation matrix in y-axis
 * @param t       pitch
 * @param matx    a 3x3 rotation matrix
 */
void roty(double t, float matx[][3]){
    float c, s;
    c = cos(t);
    s = sin(t);
    matx[0][0] = c;
    matx[0][2] = s;
    matx[2][0] = -s;
    matx[2][2] = c;
}


/** 
 * @brief Calculate rotation matrix in z-axis
 * @param t       yaw
 * @param matx    a 3x3 rotation matrix
 */
void rotz(double t, float matx[][3]){
    float c, s;
    c = cos(t);
    s = sin(t);
    matx[0][0] = c;
    matx[0][1] = -s;
    matx[1][0] = s;
    matx[1][1] = c;
}


/** 
 * @brief matrix multiplication when m=1
 * @param a    a nxk matrix
 * @param b    a kxm matrix
 * @param c    result
 * @param n    the number of rows of matrix a
 * @param k    the number of columns of matrix a
 * @param m    the number of columns of matrix b
 */
void multiply1(float a[], float b[], float c[][1], int n, int k, int m){
    for (int i = 0; i < n; i++){
        for (int j = 0; j < m; j++){
            c[i][j] = 0;
            for (int v = 0; v < k; v++){
                c[i][j] += a[i*k+v] * b[v*m+j];
            }
        }
    }
}


/** 
 * @brief matrix multiplication when m=3
 * @param a    a nxk matrix
 * @param b    a kxm matrix
 * @param c    result
 * @param n    the number of rows of matrix a
 * @param k    the number of columns of matrix a
 * @param m    the number of columns of matrix b
 */
void multiply3(float a[], float b[], float c[][3], int n, int k, int m){
    for (int i = 0; i < n; i++){
        for (int j = 0; j < m; j++){
            c[i][j] = 0;
            for (int v = 0; v < k; v++){
                c[i][j] += a[i*k+v] * b[v*m+j];
            }
        }
    }
}


/** 
 * @brief Calculate pose matrix
 * @param oxts    longitude,latitude,altitude,roll,pitch,yaw
 */
float **poses_from_oxts(Oxts oxts){
    float **p = new float *[4]; // Create a dynamic two-dimensional array p
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
    multiply3(*Ry, *Rx, c, 3, 3, 3);

    int z = 0;
    float Rz1[9],c1[9];
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            Rz1[z] = Rz[i][j];
            c1[z] = c[i][j];
            z++;
        }
    }
    multiply3(Rz1, c1, R, 3, 3, 3);

    for(int i = 0; i<3; i++){
        t[i] = t[i] - t_0[i];
    }

    // Splicing rotation matrix and translation matrix
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


/** 
 * @brief Distinguish between dynamic and static points by comparing frames.
 * @param pre_cloud    pointcloud from previous frame
 * @param cur_cloud    pointcloud for the current frame
 * @param cheader      the header of this pointcloud
 */
void seg_dynamic(PointCloudEX::Ptr pre_cloud, PointCloudEX::Ptr cur_cloud, std_msgs::Header cheader){
    int cur_num = cur_cloud->points.size(); 
    // dynamic and static pointcloud
    PointCloudEX::Ptr dyn_cloud(new PointCloudEX);
    PointCloudEX::Ptr sta_cloud(new PointCloudEX);

    // the kdtree generation method of PCL
    pcl::KdTreeFLANN<TanwayPCLEXPoint> kdtreelast;
    kdtreelast.setInputCloud(pre_cloud);
    float search_num = 1; 
    float thre_dis = 1;
    TanwayPCLEXPoint searchPoint;
    
    for (int pid = 0; pid < cur_num; pid++){
        searchPoint = cur_cloud->points[pid];
        std::vector<float> k_dis;
        std::vector<int> k_inds;
        kdtreelast.nearestKSearch(searchPoint,search_num,k_inds,k_dis);
        if(k_dis[0] < thre_dis){
            sta_cloud->push_back(searchPoint);
        }
        else{
            dyn_cloud->push_back(searchPoint);
        }
    }
    
    // publish pointclouds
    std::cout << dyn_cloud->size() << std::endl;
    std::cout << sta_cloud->size() << std::endl;
    
    sensor_msgs::PointCloud2 msg_dyn;
    pcl::toROSMsg(*dyn_cloud, msg_dyn);
    msg_dyn.header = cheader;
    pub_dyn.publish(msg_dyn);

    sensor_msgs::PointCloud2 msg_sta;
    pcl::toROSMsg(*sta_cloud, msg_sta);
    msg_sta.header = cheader;
    pub_sta.publish(msg_sta);
}


/** 
 * @brief pointcloud registration
 */
void create_pcd(){
    sensor_msgs::PointCloud2 pcl_new_msg;  // pointcloud messages waiting to be sent.
    PointCloudEX mid_pcl;   // Establishe a pointcloud of PCL as an intermediate process.
    
    int num = 0;
    sensor_msgs::PointCloud2::ConstPtr pcloud(new sensor_msgs::PointCloud2);
    PointCloudEX::Ptr cloudadd(new PointCloudEX);
    PointCloudEX::Ptr cloud(new PointCloudEX);

    PointCloudEX::Ptr lastcloud(new PointCloudEX);
    PointCloudEX::Ptr currentcloud(new PointCloudEX);
    PointCloudEX::Ptr bgcloud(new PointCloudEX);

    for(int i = 0; i < frame_num; i++){
        // Calculate pose matrix.
        float **pose;
        pose = poses_from_oxts(oxts_buffer[i]);
        /* print pose matrix */
        // std::cout << i << std::endl;
        // for(int i = 0; i < 4; i++){
        //     for(int j = 0; j < 4; j++){
        //         std::cout << pose[i][j] << ",";
        //     }
        //     std::cout << endl;
        // }
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

        // pointcloud registration
        int pc_row;
        double x,y;
        if(num == 0){
            pcloud = lidar_buffer[num];
            pcl::fromROSMsg(*pcloud, *cloudadd);
            pc_row = cloudadd->points.size();
            float pc_array[4];
            for(int r = 0; r < pc_row; r++){
                pc_array[0] = cloudadd->points[r].x;
                pc_array[1] = cloudadd->points[r].y;
                pc_array[2] = cloudadd->points[r].z;
                pc_array[3] = 1.0;
                float pc[4][1] = {0};
                multiply1(pose1, pc_array, pc, 4, 4, 1);
                x = pc[0][0];
                y = pc[1][0];
                cloudadd->points[r].z = pc[2][0];
                cloudadd->points[r].x = x * cos(yaw) + y * sin(yaw);
                cloudadd->points[r].y = -x * sin(yaw) + y * cos(yaw);
            }
            *lastcloud = *cloudadd;
        }
        else if(num > 0){
            pcloud = lidar_buffer[num];
            pcl::fromROSMsg(*pcloud, *cloud);
            pc_row = cloud->points.size();
            float pc_array[4];
            for(int r = 0; r < pc_row; r++){
                pc_array[0] = cloud->points[r].x;
                pc_array[1] = cloud->points[r].y;
                pc_array[2] = cloud->points[r].z;
                pc_array[3] = 1.0;
                float pc[4][1] = {0};
                multiply1(pose1, pc_array, pc, 4, 4, 1);
                x = pc[0][0];
                y = pc[1][0];
                cloud->points[r].z = pc[2][0];
                cloud->points[r].x = x * cos(yaw) + y * sin(yaw);
                cloud->points[r].y = -x * sin(yaw) + y * cos(yaw);
            }
            if(num == frame_num - 1){
                *currentcloud = * cloud;
                seg_dynamic(lastcloud, currentcloud, pcloud->header);
            }
            *cloudadd = *cloudadd + *cloud;
            cloud->clear();
        }
        num += 1;
    }
    mid_pcl = *cloudadd;
    pcl::toROSMsg(mid_pcl, pcl_new_msg);  // Converting point clouds into messages before publishing.
    pub_pcl.publish(pcl_new_msg); // Publish adjusted pointcloud data with the theme 'new_cloud'.
}


/** 
 * @brief Callback function for data storage
 * @param pclMsg    pointcloud message
 * @param gpsMsg    gps message
 * @param imuMsg    imu message
 */
void callback(const sensor_msgs::PointCloud2::ConstPtr& pclMsg, const sensor_msgs::NavSatFix::ConstPtr& gpsMsg, const sensor_msgs::Imu::ConstPtr& imuMsg){
    cnt++;
    std::cout << cnt << std::endl; 
    Oxts oxt;
    
    if(lidar_buffer.size()==frame_num){
        lidar_buffer.pop_front();
    }
    lidar_buffer.push_back(pclMsg);
    // gps
    sensor_msgs::NavSatFix::Ptr gpsmsg(new sensor_msgs::NavSatFix(*gpsMsg));
    oxt.lon = gpsmsg->longitude;
    oxt.lat = gpsmsg->latitude;
    oxt.alt = gpsmsg->altitude;
    // imu
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
    // Store data in the queue.
    if(oxts_buffer.size() < frame_num){
        oxts_buffer.push_back(oxt);
    }
    else if(oxts_buffer.size() == frame_num){
        oxts_buffer.pop_front();
        oxts_buffer.push_back(oxt);
        // calculate t_0
        scale = cos(oxt.lat * M_PI / 180);
        t_0[0] = scale * oxt.lon * M_PI * er / 180;
        t_0[1] = scale * er * log(tan((90 + oxt.lat) * M_PI / 360));
        t_0[2] = oxt.alt;
        create_pcd();
    }
}

int main(int argc, char** argv){
    // Initialize ROS node.
    ros::init(argc, argv, "play_bag_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pcl(nh, topic_pcl, 10);
    message_filters::Subscriber<sensor_msgs::NavSatFix> sub_gps(nh, topic_gps, 10);
    message_filters::Subscriber<sensor_msgs::Imu> sub_imu(nh, topic_imu, 10);

    // Synchronize data from three topics
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::NavSatFix, sensor_msgs::Imu> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_pcl, sub_gps, sub_imu);
    // Specify a callback function to achieve synchronous acquisition of data from three topics. 
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    
    // Establish  publishers to publish adjusted pointclouds.
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/new_cloud", 1000);  
    // pub = nh.advertise<sensor_msgs::PointCloud2>("foreground", 10);
    // pub_bg = nh.advertise<sensor_msgs::PointCloud2>("background", 10);
    pub_dyn = nh.advertise<sensor_msgs::PointCloud2>("dyncloud", 1000);
    pub_sta = nh.advertise<sensor_msgs::PointCloud2>("stacloud", 1000);

    ros::spin();

    return 0;
}
