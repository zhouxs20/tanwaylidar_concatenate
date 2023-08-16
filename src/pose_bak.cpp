#include <iostream>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <std_msgs/String.h>
#include <rosbag/view.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "tf/transform_datatypes.h"//转换函数头文件
#include <geometry_msgs/Point32.h>//geometry_msgs消息数据类型
#include <tf/tf.h>
#include <dirent.h>
#include <cmath>
#include <Eigen/Geometry> 

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>


using namespace std;

double t_0[3] = {0,0,0};
int flag = 0;
double scale = 0;

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
    //按数值大小的方式  避免 1.txt 10.txt 11.txt的方式进行排序 
    // sort(filenames.begin(), filenames.end(), [](string a, string b){
    //     return stoi(a) < stoi(b);
    // });
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
// float **transform_from_rot_trans(float R[], float t[]){
//     float **p = new float *[4]; // 创建动态二维数组p
//     for(int i = 0; i < 4; i++){
//         p[i] = new float[4];
//     }
//     for(int i = 0; i < 3; i++){
//         for (int j = 0; j < 3; j++){
//             p[i][j] = R[i*4+j];
//         }
//     }
//     for(int i = 0; i < 3; i++){
//         p[i][3] = t[i];
//     }
//     float a[4] = {0, 0, 0, 1};
//     for(int j = 0; j < 4; j++){
//         p[3][j] = a[j];
//     }
//     return p;
// }

float **poses_from_oxts(float oxts[6]){
    float **p = new float *[4]; // 创建动态二维数组p
    for(int i = 0; i < 4; i++){
        p[i] = new float[4];
    }
    double er = 6378137.0000;
    double tx, ty, tz;
    tx = scale * oxts[0] * M_PI * er / 180;
    ty = scale * er * log(tan((90 + oxts[1]) * M_PI / 360));
    tz = oxts[2];
    double t[3] = {tx, ty, tz};
    if(flag == 0){
        for (int i=0;i<3;i++){
            t_0[i] = t[i];
        }
        flag = 1;
    }
    float Rx[3][3] = {{1, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    float Ry[3][3] = {{0, 0, 0}, {0, 1, 0}, {0, 0, 0}};
    float Rz[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 1}};
    rotx(oxts[3], Rx);
    roty(oxts[4], Ry);
    rotz(oxts[5], Rz);
    
    float **c, **R;
    int z = 0;
    c = Multiply(*Ry, *Rx, 3, 3, 3);
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

void create_pcd(string path, float oxts[][6], int f, int row){
    std::vector<std::string> filenames;
    // 读取文件名并排序
    GetFileNames(path, filenames, "pcd");
    for(int i = 0; i < filenames.size(); ++i) {
        std::cout << filenames[i] << std::endl;
    }
    int num = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudadd(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(int i = 0; i < row; i++){
        float **pose;
        pose = poses_from_oxts(oxts[i]);
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
        int pc_row, pc_col;

        // 以下为pcd点云处理部分
        if(num == 0){
            string pcd_path;
            pcd_path += path;
            pcd_path += filenames[0];
            pcl::io::loadPCDFile(pcd_path, *cloudadd);
            pc_row = cloudadd->points.size();
            std::cout << pc_row << std::endl;
            std::cout << cloudadd->points[pc_row] << std::endl;

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
            string pcd_path;
            pcd_path += path;
            pcd_path += filenames[num];
            pcl::io::loadPCDFile(pcd_path, *cloud);
            pc_row = cloud->points.size();
            
            float pc_array[4];
            for(int r = 0; r < pc_row; r++){
                pc_array[0] = cloud->points[r].x;
                pc_array[1] = cloud->points[r].y;
                pc_array[2] = cloud->points[r].z;
                pc_array[3] = 1;
                float **pc;
                pc = Multiply(pose1, pc_array, 4, 4, 1);
                cloud->points[r].x = pc[0][0];
                cloud->points[r].y = pc[1][0];
                cloud->points[r].z = pc[2][0];
            }
            *cloudadd = *cloudadd + *cloud;
            
            cloud->clear();
        }
        num += f;
    }
    pcl::io::savePCDFileBinary("mytest_pcd.pcd", *cloudadd);
    pcl::visualization::PCLVisualizer viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloudadd, 0, 255, 0); // rgb
	viewer.addPointCloud(cloudadd, green, "cloudadd"); 
    viewer.spin();
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
            std::cout << "imu null " << std::endl; 
        } 
        else {
            std::cout << "imu stamp:" << imu->header.stamp << std::endl;
            double ortx, orty, ortz, ortw;
            ortx = imu->orientation.x;
            orty = imu->orientation.y;
            ortz = imu->orientation.z;
            ortw = imu->orientation.w;
            double roll, pitch, yaw;
            imu_to_rpy(ortx, orty, ortz, ortw, &roll, &pitch, & yaw);
            imu_res[imu_num][0] = roll;
            imu_res[imu_num][1] = pitch;
            imu_res[imu_num][2] = yaw;
            imu_num++;
        }
        sensor_msgs::NavSatFix::ConstPtr gps = m.instantiate<sensor_msgs::NavSatFix>();
        if (gps == nullptr) {
            std::cout << "gps null " << std::endl; 
        } 
        else {
            std::cout << "gps stamp:" << gps->header.stamp << std::endl; 
            gps_res[gps_num][0] = gps->longitude;
            gps_res[gps_num][1] = gps->latitude;
            gps_res[gps_num][2] = gps->altitude;
            gps_num++; //29760
        }
    }

    // 关闭bag文件
    bag.close();

    string path = "/home/zxs/test_bag1/pcd/";
    float oxts[30000][6] = {0};
    int k = 0;
    for(int i = 0; i <= gps_num; i++){
        if(i % 10 == 0){
            oxts[k][0] = float(gps_res[i][0]);
            oxts[k][1] = float(gps_res[i][1]);
            oxts[k][2] = float(gps_res[i][2]);
            oxts[k][3] = float(imu_res[i][0]);
            oxts[k][4] = float(imu_res[i][1]);
            oxts[k][5] = float(imu_res[i][2]);
            k++;
        }
    }
    scale = cos(oxts[0][1] * M_PI / 180);
    int frame = 1;
    create_pcd(path, oxts, frame, 10); //叠加前10帧
    return 0;
}