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


using namespace std;

double t_0[3] = {0,0,0};
int flag = 0;

// // 将imu四元数转换为欧拉角
// void imu_to_rpy(double x, double y, double z, double w, double* roll, double* pitch, double* yaw)
// {   
//     std::cout << "x="<<x << ",y="<<y<<",z="<<z<<",w="<<w<<std::endl;
//     tf::Quaternion q(x,y,z,w);
//     tf::Matrix3x3(q).getRPY(*roll, *pitch, *yaw);
//     std::cout << "roll="<< *roll << ",pitch="<<*pitch<<",yaw="<< *yaw <<std::endl;
//     // *roll = *roll * 180 / M_PI;
//     // *pitch = *pitch * 180 / M_PI;
//     // *yaw = *yaw * 180 / M_PI;
// }


void imu_to_rpy(double x, double y, double z, double w, double* roll, double* pitch, double* yaw){
   

    *roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    *pitch = asin(2 * (w * y - z * x));
    *yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

}

// def four2ola(x,y,z,w):
//     x,y,z,w = float(x),float(y),float(z),float(w)
//     roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
//     pitch = math.asin(2 * (w * y - z * x))
//     yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
//     return roll,pitch,yaw

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
    // sort(filenames.begin(), filenames.end());
    //按数值大小的方式  避免 1.txt 10.txt 11.txt的方式进行排序 
    sort(filenames.begin(), filenames.end(), [](string a, string b){
        return pcdname2int(a,'.') < pcdname2int(b,'.');
    });
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


float **poses_from_oxts(float oxts[6]){
    std::cout << "lon: " << oxts[0] << ",lat = " << oxts[1] << ", alt = " <<  oxts[2] << std::endl;
    std::cout << "roll: " << oxts[3] << ",pitch = " << oxts[4] << ", yaw = " <<  oxts[5] << std::endl;

    float **p = new float *[4]; // 创建动态二维数组p
    for(int i = 0; i < 4; i++){
        p[i] = new float[4];
    }
    double er = 6378137.0000;
    double scale = cos(oxts[1] * M_PI / 180);
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
    std::cout << "tx_0: " << t_0[0] << ",ty_0 = " << t_0[1] << ", tz_0 = " <<  t_0[2] << std::endl;
    std::cout << "tx: " << t[0] << ",ty = " << t[1] << ", tz = " <<  t[2] << std::endl;
    float Rx[3][3] = {{1, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    float Ry[3][3] = {{0, 0, 0}, {0, 1, 0}, {0, 0, 0}};
    float Rz[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 1}};
    rotx(oxts[3], Rx);
    roty(oxts[4], Ry);
    rotz(oxts[5], Rz);
    
    float **c, **R;
    c = Multiply(*Ry, *Rx, 3, 3, 3);

    

    std::cout << "Rz" << std::endl;
    float Rz_v[9];
    for(int ii = 0; ii < 3; ii++){
        for(int jj = 0; jj < 3; jj++){
            std::cout << Rz[ii][jj] << ",";
            Rz_v[ii*3+jj] = Rz[ii][jj];
        }
        std::cout << endl;
    }


    std::cout << "c" << std::endl;
    float c_v[9];
    for(int ii = 0; ii < 3; ii++){
        for(int jj = 0; jj < 3; jj++){
            std::cout << c[ii][jj] << ",";
            c_v[ii*3+jj] = c[ii][jj];
        }
        std::cout << endl;
    }

    R = Multiply(Rz_v, c_v, 3, 3, 3);
    std::cout << "R" << std::endl;
    for(int ii = 0; ii < 3; ii++){
        for(int jj = 0; jj < 3; jj++){
            std::cout << R[ii][jj] << ",";
        }
        std::cout << endl;
    }
    
    double dt[3];
    for(int i = 0; i<3; i++){
        dt[i] = t[i] - t_0[i];
    }
    std::cout << "dx: " << dt[0] << ",dy = " << dt[1] << ",dz = " <<  dt[2] << std::endl;

    for(int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            p[i][j] = R[i][j];
        }
    }
    for(int i = 0; i < 3; i++){
        p[i][3] = dt[i];
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

    int num = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(int i = 0; i < row; i++){
        std::cout << i << std::endl;
        float **pose;
        pose = poses_from_oxts(oxts[i]);//计算结果存在问题
        
        
        
        // 以下为pcd点云处理部分
  
        string pcd_path;
        pcd_path += path;
        pcd_path += filenames[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(pcd_path, *pcl_cloud);
        std::cout << "cloud size = " <<pcl_cloud->points.size() << std::endl;
        for (const auto& pcl_p : pcl_cloud->points) {
            float lidar_loc[4] = {pcl_p.x,pcl_p.y,pcl_p.z,1.0f};
            float pose_vec[16];
            for (int r=0;r<4;r++){
                for (int c=0;c<4;c++){
                    pose_vec[r*4+c] = pose[r][c];
                }
            }
            // for (int r=0;r<4;r++){
            //     delete [] pose[r];
            // }
            // delete [] pose;
            float** point_enu_loc = Multiply(pose_vec,lidar_loc,4,4,1);
            pcl::PointXYZ n_p;
            n_p.x = point_enu_loc[0][0];
            n_p.y = point_enu_loc[1][0];
            n_p.z = point_enu_loc[2][0];
            // std::cout << "x=" << pcl_p.x<<" ,y=" << pcl_p.y << ",z=" <<pcl_p.z << std::endl;
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
    int frame = 1;
    create_pcd(path, oxts, frame, 10); //叠加前100帧
    return 0;
}