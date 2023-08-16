#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/String.h>
#include <rosbag/view.h>
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/NavSatFix.h"
#include "tf/transform_datatypes.h"//转换函数头文件
#include <geometry_msgs/Point32.h>//geometry_msgs消息数据类型
#include <tf/tf.h>
#include <dirent.h>
#include <cmath>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>


#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;

double t_0[3] = {0.0,0.0,0.0};
int flag = 0;


std::vector<std::string> stringSplit(const std::string& str, char delim) {
    std::size_t previous = 0;
    std::size_t current = str.find(delim);
    std::vector<std::string> elems;
    while (current != std::string::npos) {
        if (current > previous) {
            elems.push_back(str.substr(previous, current - previous));
        }
        previous = current + 1;
        current = str.find(delim, previous);
    }
    if (previous != str.size()) {
        elems.push_back(str.substr(previous));
    }
    return elems;
}

int pcdname2int(std::string& str, char delim){
    std::vector<std::string> string_vec = stringSplit(str, delim);
    int int1 = std::stoi(string_vec[0]) - 1670000000;
    int int2 = std::stoi(string_vec[1])/100000000;
    int res = int1 *100 + int2;
    return res ;
}

void GetFileNames(string path, vector<string> &filenames)
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
    // sort(filenames.begin(), filenames.end());
    //按数值大小的方式  避免 1.txt 10.txt 11.txt的方式进行排序 
    sort(filenames.begin(), filenames.end(), [](string a, string b){
        return pcdname2int(a,'.') < pcdname2int(b,'.');
    });
}

void imu_to_rpy(double x, double y, double z, double w, double* roll, double* pitch, double* yaw){

    *roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    *pitch = asin(2 * (w * y - z * x));
    *yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

}



//矩阵相乘函数
double **Multiply(double a[], double b[], int n, int k, int m){
    double **c = new double *[n]; // 创建动态二维数组c
    for(int i = 0; i < n; i++){
        c[i] = new double[m];
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



class GpsImuNavigation
{
public:
    GpsImuNavigation(double longitude, double latitude,double altitude){
        double ecef_x0=1,ecef_y0=1,ecef_z0=1;
        this->lla2ecef(longitude, latitude, altitude, ecef_x0, ecef_y0, ecef_z0);
        double lon_rad = longitude * M_PI / 180;
        double lat_rad = latitude * M_PI / 180;

        Eigen::Matrix4d ecef2enu_translation;
        ecef2enu_translation.setIdentity();
        ecef2enu_translation(0,3) = -ecef_x0;
        ecef2enu_translation(1,3) = -ecef_y0;
        ecef2enu_translation(2,3) = -ecef_z0;

        Eigen::Matrix4d ecef2enu_rotate;
        ecef2enu_rotate.setIdentity();
        ecef2enu_rotate(0,0) = -sin(lon_rad);
        ecef2enu_rotate(0,1) = cos(lon_rad);
        ecef2enu_rotate(1,0) = -sin(lat_rad) * cos(lon_rad);
        ecef2enu_rotate(1,1) = -sin(lat_rad)* sin(lon_rad);
        ecef2enu_rotate(1,2) = cos(lat_rad);
        ecef2enu_rotate(2,0) = cos(lat_rad) * cos(lon_rad);
        ecef2enu_rotate(2,1) = cos(lat_rad)* sin(lon_rad);
        ecef2enu_rotate(2,2) = sin(lat_rad);
        ecef2enu_matrix = ecef2enu_rotate * ecef2enu_translation;
        std::cout << "ecef2enu_matrix :" << std::endl;
        std::cout << ecef2enu_matrix << std::endl;
    }

    void lla2ecef(double longitude, double latitude,double altitude, double& ecef_x,double& ecef_y,double& ecef_z){
        double lon = longitude * M_PI / 180;
        double lat = latitude * M_PI / 180;
        double alt = altitude * M_PI / 180;
        double cosLon = cos(lon);
        double cosLat = cos(lat);
        double sinLon = sin(lon);
        double sinLat = sin(lat);
        
        double N = wgs84_a / sqrt(1.0 - pow_e_2 * sinLat * sinLat);
        ecef_x = (N + alt) * cosLat * cosLon;
        ecef_y = (N + alt) * cosLat * sinLon;
        ecef_z = (N * (1.0 - pow_e_2) + alt) * sinLat;
    }

    void ecef2enu(double ecef_x,double ecef_y,double ecef_z,double& enu_x,double& enu_y,double& enu_z ){
        Eigen::Vector4d ecef_location;
        ecef_location << ecef_x, ecef_y, ecef_z, 1.0;
        Eigen::Matrix<double,4,1> enu_location = ecef2enu_matrix * ecef_location;
        enu_x = enu_location(0,0);
        enu_y = enu_location(1,0);
        enu_z = enu_location(2,0);
    }

    Eigen::Matrix4d point_cloud_convert_matrix(double longitude, double latitude, double altitude, double roll, double pitch, double yaw){
        double ecef_x=0,ecef_y=0,ecef_z=0;
        double enu_x=0,enu_y=0,enu_z=0;
        this->lla2ecef(longitude,latitude,altitude,ecef_x,ecef_y,ecef_z);
        this->ecef2enu(ecef_x,ecef_y,ecef_z,enu_x,enu_y,enu_z);
        Eigen::Matrix3d r_x = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d r_y = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d r_z = Eigen::Matrix3d::Identity();
        r_x(1,1) = cos(roll);
        r_x(1,2) = -sin(roll);
        r_x(2,1) = sin(roll);
        r_x(2,2) = cos(roll);
        r_y(0,0) = cos(pitch);
        r_y(0,2) = sin(pitch);
        r_y(2,0) = -sin(pitch);
        r_y(2,2) = cos(pitch);

        r_z(0,0) = cos(yaw);
        r_z(0,1) = -sin(yaw);
        r_z(1,0) = sin(yaw);
        r_z(1,1) = cos(yaw);

        Eigen::Matrix3d R = r_z * r_y * r_x;

        Eigen::Matrix4d location_roation_matrix = Eigen::Matrix4d::Identity();
        location_roation_matrix.row(0) << R(0,0), R(0,1), R(0,2), enu_x;
        location_roation_matrix.row(1) << R(1,0), R(1,1), R(1,2), enu_y;
        location_roation_matrix.row(2) << R(2,0), R(2,1), R(2,2), enu_z;
        

        std::cout << "point cloud rotate matrix" << std::endl;
        std::cout << location_roation_matrix << std::endl;
        return location_roation_matrix;

    }


private:
    double wgs84_a = 6378137.0000;
    double wgs84_b = 6356752.3142;
    double wgs84_f = (wgs84_a - wgs84_b) / wgs84_a;
    double pow_e_2 = wgs84_f * (2.0 - wgs84_f);
    Eigen::Matrix4d ecef2enu_matrix;
};





void create_pcd(string path, double oxts[][6], int f, int row){
    std::vector<std::string> filenames;
    // 读取文件名并排序
    GetFileNames(path, filenames);
    
    GpsImuNavigation gps_imu_navigation{oxts[0][0],oxts[0][1],oxts[0][2]}; // 第一个位置的坐标作为enu的原点
    
    int num = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(int i = 0; i < row; i++){

        Eigen::Matrix4d location_roation_matrix = gps_imu_navigation.point_cloud_convert_matrix(oxts[i][0],oxts[i][1],oxts[i][2],oxts[i][3],oxts[i][4],oxts[i][5]);


        std::cout << i << std::endl;

        
    //     // 以下为pcd点云处理部分
  
        string pcd_path;
        pcd_path += path;
        pcd_path += filenames[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(pcd_path, *pcl_cloud);
        std::cout << "cloud size = " <<pcl_cloud->points.size() << std::endl;
        for (const auto& pcl_p : pcl_cloud->points) {
            Eigen::Vector4d lidar_loc ;
            lidar_loc << pcl_p.x,pcl_p.y,pcl_p.z,1.0 ;
         
        
            Eigen::Matrix<double,4,1> point_enu_loc = location_roation_matrix * lidar_loc;
            pcl::PointXYZ n_p;
            n_p.x = point_enu_loc(0,0);
            n_p.y = point_enu_loc(1,0);
            n_p.z = point_enu_loc(2,0);

            result_cloud->push_back(n_p);
        }
      

        num += f;
    }
    pcl::io::savePCDFileBinary("test_pcd.pcd", *result_cloud);

}



int main(int argc, char** argv){
    // 测试ros输出
    ROS_INFO("hello");

    // 初始化ROS节点
    ros::init(argc, argv, "play_bag_node");
    ros::NodeHandle nh;
 
    // 创建ROSbag对象，打开bag文件
    rosbag::Bag bag;
    bag.open("/home/zxs/test_bag1/2023_02_21_102_2023-02-21-14-10-41_0.bag", rosbag::bagmode::Read);
    
    // 设置需要遍历的topic
    std::vector<std::string> topics;
    topics.push_back(std::string("/gps"));
    topics.push_back(std::string("/imu"));
    // for (std::vector<std::string>::const_iterator i = topics.begin(); i != topics.end(); i++) {
    // std::cout << *i << ' '; // 输出topics
    // }   
    
    //创建view，用于读取bag中的topic
    rosbag::View view(bag, rosbag::TopicQuery(topics)); // 读指定的topic
    //rosbag::View view_all(bag); //读取全部topic

    // 读取gps和imu数据，相当于python中的gps_imu函数
    int imu_num = 0;
    int gps_num = 0;
    // 用二维数组来存gps和imu数据
    double imu_res[30000][3] = {};
    double gps_res[30000][3] = {};
    for (auto m : view) {
        sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
        if (imu == nullptr) {
            // std::cout << "imu null " << std::endl; //非必要输出
        } 
        else {
            // std::cout << "imu stamp:" << imu->header.stamp << std::endl; //非必要输出
            double ortx, orty, ortz, ortw;
            ortx = imu->orientation.x;
            orty = imu->orientation.y;
            ortz = imu->orientation.z;
            ortw = imu->orientation.w;
            double roll=0, pitch=0, yaw=0;
            imu_to_rpy(ortx, orty, ortz, ortw, &roll, &pitch, &yaw);
            imu_res[imu_num][0] = roll;
            imu_res[imu_num][1] = pitch;
            imu_res[imu_num][2] = yaw;
            imu_num++;
        }
        sensor_msgs::NavSatFix::ConstPtr gps = m.instantiate<sensor_msgs::NavSatFix>();
        if (gps == nullptr) {
            // std::cout << "gps null " << std::endl; //非必要输出
        } 
        else {
            // std::cout << "gps stamp:" << gps->header.stamp << std::endl; //非必要输出
            gps_res[gps_num][0] = gps->longitude;
            gps_res[gps_num][1] = gps->latitude;
            gps_res[gps_num][2] = gps->altitude;
            gps_num++; //29760
        }
    }

    // 关闭bag文件
    bag.close();

    string path = "/home/zxs/test_bag1/pcd/";
    double oxts[30000][6] = {0};
    int k = 0;
    for(int i = 0; i <= gps_num; i++){
        if(i % 10 == 0){
            oxts[k][0] = (double)(gps_res[i][0]);
            oxts[k][1] = (double)(gps_res[i][1]);
            oxts[k][2] = (double)(gps_res[i][2]);
            oxts[k][3] = (double)(imu_res[i][0]);
            oxts[k][4] = (double)(imu_res[i][1]);
            oxts[k][5] = (double)(imu_res[i][2]);
            k++;
        }
    }
    int frame = 1;
    create_pcd(path, oxts, frame, 10); //叠加前100帧
    return 0;
}