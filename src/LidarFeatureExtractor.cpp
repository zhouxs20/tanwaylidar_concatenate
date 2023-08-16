#include "LidarFeatureExtractor/LidarFeatureExtractor.h"

LidarFeatureExtractor::LidarFeatureExtractor(int n_scans,int NumCurvSize,float DistanceFaraway,int NumFlat,
                                             int PartNum,float FlatThreshold,float BreakCornerDis,float LidarNearestDis,float KdTreeCornerOutlierDis)
                                             :N_SCANS(n_scans),
                                              thNumCurvSize(NumCurvSize),
                                              thDistanceFaraway(DistanceFaraway),
                                              thNumFlat(NumFlat),
                                              thPartNum(PartNum),
                                              thFlatThreshold(FlatThreshold),
                                              thBreakCornerDis(BreakCornerDis),
                                              thLidarNearestDis(LidarNearestDis){
  vlines.resize(N_SCANS);
  for(auto & ptr : vlines){
    ptr.reset(new pcl::PointCloud<PointType>());
  }
  vcorner.resize(N_SCANS);
  vsurf.resize(N_SCANS);
}

bool LidarFeatureExtractor::plane_judge(const std::vector<PointType>& point_list,const int plane_threshold)
{
  int num = point_list.size();
  float cx = 0;
  float cy = 0;
  float cz = 0;
  for (int j = 0; j < num; j++) {
    cx += point_list[j].x;
    cy += point_list[j].y;
    cz += point_list[j].z;
  }
  cx /= num;
  cy /= num;
  cz /= num;
  //mean square error
  float a11 = 0;
  float a12 = 0;
  float a13 = 0;
  float a22 = 0;
  float a23 = 0;
  float a33 = 0;
  for (int j = 0; j < num; j++) {
    float ax = point_list[j].x - cx;
    float ay = point_list[j].y - cy;
    float az = point_list[j].z - cz;

    a11 += ax * ax;
    a12 += ax * ay;
    a13 += ax * az;
    a22 += ay * ay;
    a23 += ay * az;
    a33 += az * az;
  }
  a11 /= num;
  a12 /= num;
  a13 /= num;
  a22 /= num;
  a23 /= num;
  a33 /= num;

  Eigen::Matrix< double, 3, 3 > _matA1;
  _matA1.setZero();
  Eigen::Matrix< double, 3, 1 > _matD1;
  _matD1.setZero();
  Eigen::Matrix< double, 3, 3 > _matV1;
  _matV1.setZero();

  _matA1(0, 0) = a11;
  _matA1(0, 1) = a12;
  _matA1(0, 2) = a13;
  _matA1(1, 0) = a12;
  _matA1(1, 1) = a22;
  _matA1(1, 2) = a23;
  _matA1(2, 0) = a13;
  _matA1(2, 1) = a23;
  _matA1(2, 2) = a33;

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(_matA1, Eigen::ComputeThinU | Eigen::ComputeThinV);
  _matD1 = svd.singularValues();
  _matV1 = svd.matrixU();
  if (_matD1(0, 0) < plane_threshold * _matD1(1, 0)) {
    return true;
  }
  else{
    return false;
  }
}

void LidarFeatureExtractor::detectFeaturePoint(pcl::PointCloud<PointType>::Ptr& cloud,
                                                std::vector<int>& pointsLessSharp,
                                                std::vector<int>& pointsLessFlat){
  int CloudFeatureFlag[20000];
  float cloudCurvature[20000];
  float cloudDepth[20000];
  int cloudSortInd[20000];
  float cloudReflect[20000];
  int reflectSortInd[20000];
  int cloudAngle[20000];

  pcl::PointCloud<PointType>::Ptr& laserCloudIn = cloud;

  int cloudSize = laserCloudIn->points.size();

  PointType point;
  pcl::PointCloud<PointType>::Ptr _laserCloud(new pcl::PointCloud<PointType>());
  _laserCloud->reserve(cloudSize);

  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn->points[i].x;
    point.y = laserCloudIn->points[i].y;
    point.z = laserCloudIn->points[i].z;
#ifdef UNDISTORT
    point.normal_x = laserCloudIn.points[i].normal_x;
#else
    point.normal_x = 1.0;
#endif
    point.intensity = laserCloudIn->points[i].intensity;

    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    _laserCloud->push_back(point);
    CloudFeatureFlag[i] = 0;
  }

  cloudSize = _laserCloud->size();

  int debugnum1 = 0;
  int debugnum2 = 0;
  int debugnum3 = 0;
  int debugnum4 = 0;
  int debugnum5 = 0;

  int count_num = 1;
  bool left_surf_flag = false;
  bool right_surf_flag = false;

  //---------------------------------------- surf feature extract ---------------------------------------------
  int scanStartInd = 5;
  int scanEndInd = cloudSize - 6;

  int thDistanceFaraway_fea = 0;

  for (int i = 5; i < cloudSize - 5; i ++ ) {

    float diffX = 0;
    float diffY = 0;
    float diffZ = 0;

    float dis = sqrt(_laserCloud->points[i].x * _laserCloud->points[i].x +
                     _laserCloud->points[i].y * _laserCloud->points[i].y +
                     _laserCloud->points[i].z * _laserCloud->points[i].z);

    Eigen::Vector3d pt_last(_laserCloud->points[i-1].x, _laserCloud->points[i-1].y, _laserCloud->points[i-1].z);
    Eigen::Vector3d pt_cur(_laserCloud->points[i].x, _laserCloud->points[i].y, _laserCloud->points[i].z);
    Eigen::Vector3d pt_next(_laserCloud->points[i+1].x, _laserCloud->points[i+1].y, _laserCloud->points[i+1].z);

    double angle_last = (pt_last-pt_cur).dot(pt_cur) / ((pt_last-pt_cur).norm()*pt_cur.norm());
    double angle_next = (pt_next-pt_cur).dot(pt_cur) / ((pt_next-pt_cur).norm()*pt_cur.norm());
 
    if (dis > thDistanceFaraway || (fabs(angle_last) > 0.966 && fabs(angle_next) > 0.966)) {
      thNumCurvSize = 2;
    } else {
      thNumCurvSize = 3;
    }

    if(fabs(angle_last) > 0.966 && fabs(angle_next) > 0.966) {
      cloudAngle[i] = 1;
    }

    float diffR = -2 * thNumCurvSize * _laserCloud->points[i].intensity;
    for (int j = 1; j <= thNumCurvSize; ++j) {
      diffX += _laserCloud->points[i - j].x + _laserCloud->points[i + j].x;
      diffY += _laserCloud->points[i - j].y + _laserCloud->points[i + j].y;
      diffZ += _laserCloud->points[i - j].z + _laserCloud->points[i + j].z;
      diffR += _laserCloud->points[i - j].intensity + _laserCloud->points[i + j].intensity;
    }
    diffX -= 2 * thNumCurvSize * _laserCloud->points[i].x;
    diffY -= 2 * thNumCurvSize * _laserCloud->points[i].y;
    diffZ -= 2 * thNumCurvSize * _laserCloud->points[i].z;

    cloudDepth[i] = dis;
    cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;// / (2 * thNumCurvSize * dis + 1e-3);
    cloudSortInd[i] = i;
    cloudReflect[i] = diffR;
    reflectSortInd[i] = i;

  }

  for (int j = 0; j < thPartNum; j++) {
    int sp = scanStartInd + (scanEndInd - scanStartInd) * j / thPartNum;
    int ep = scanStartInd + (scanEndInd - scanStartInd) * (j + 1) / thPartNum - 1;

    // sort the curvatures from small to large
    for (int k = sp + 1; k <= ep; k++) {
      for (int l = k; l >= sp + 1; l--) {
        if (cloudCurvature[cloudSortInd[l]] <
            cloudCurvature[cloudSortInd[l - 1]]) {
          int temp = cloudSortInd[l - 1];
          cloudSortInd[l - 1] = cloudSortInd[l];
          cloudSortInd[l] = temp;
        }
      }
    }

    // sort the reflectivity from small to large
    for (int k = sp + 1; k <= ep; k++) {
      for (int l = k; l >= sp + 1; l--) {
        if (cloudReflect[reflectSortInd[l]] <
            cloudReflect[reflectSortInd[l - 1]]) {
          int temp = reflectSortInd[l - 1];
          reflectSortInd[l - 1] = reflectSortInd[l];
          reflectSortInd[l] = temp;
        }
      }
    }

    int smallestPickedNum = 1;
    int sharpestPickedNum = 1;
    for (int k = sp; k <= ep; k++) {
      int ind = cloudSortInd[k];

      if (CloudFeatureFlag[ind] != 0) continue;

      if (cloudCurvature[ind] < thFlatThreshold * cloudDepth[ind] * thFlatThreshold * cloudDepth[ind]) {
        
        CloudFeatureFlag[ind] = 3;

        for (int l = 1; l <= thNumCurvSize; l++) {
          float diffX = _laserCloud->points[ind + l].x -
                        _laserCloud->points[ind + l - 1].x;
          float diffY = _laserCloud->points[ind + l].y -
                        _laserCloud->points[ind + l - 1].y;
          float diffZ = _laserCloud->points[ind + l].z -
                        _laserCloud->points[ind + l - 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.02 || cloudDepth[ind] > thDistanceFaraway) {
            break;
          }

          CloudFeatureFlag[ind + l] = 1;
        }
        for (int l = -1; l >= -thNumCurvSize; l--) {
          float diffX = _laserCloud->points[ind + l].x -
                        _laserCloud->points[ind + l + 1].x;
          float diffY = _laserCloud->points[ind + l].y -
                        _laserCloud->points[ind + l + 1].y;
          float diffZ = _laserCloud->points[ind + l].z -
                        _laserCloud->points[ind + l + 1].z;
          if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.02 || cloudDepth[ind] > thDistanceFaraway) {
            break;
          }

          CloudFeatureFlag[ind + l] = 1;
        }
      }
    }
    
    for (int k = sp; k <= ep; k++) {
      int ind = cloudSortInd[k];
      if(((CloudFeatureFlag[ind] == 3) && (smallestPickedNum <= thNumFlat)) || 
          ((CloudFeatureFlag[ind] == 3) && (cloudDepth[ind] > thDistanceFaraway)) ||
          cloudAngle[ind] == 1){
        smallestPickedNum ++;
        CloudFeatureFlag[ind] = 2;
        if(cloudDepth[ind] > thDistanceFaraway) {
          thDistanceFaraway_fea++;
        }
      }

      int idx = reflectSortInd[k];
      if(cloudCurvature[idx] < 0.7 * thFlatThreshold * cloudDepth[idx] * thFlatThreshold * cloudDepth[idx]
         && sharpestPickedNum <= 3 && cloudReflect[idx] > 20.0){
        sharpestPickedNum ++;
        CloudFeatureFlag[idx] = 300;
      }
    }
    
  }

  //---------------------------------------- line feature where surfaces meet -------------------------------------
  for (int i = 5; i < cloudSize - 5; i += count_num ) {
    float depth = sqrt(_laserCloud->points[i].x * _laserCloud->points[i].x +
                       _laserCloud->points[i].y * _laserCloud->points[i].y +
                       _laserCloud->points[i].z * _laserCloud->points[i].z);
    //left curvature
    float ldiffX =
            _laserCloud->points[i - 4].x + _laserCloud->points[i - 3].x
            - 4 * _laserCloud->points[i - 2].x
            + _laserCloud->points[i - 1].x + _laserCloud->points[i].x;

    float ldiffY =
            _laserCloud->points[i - 4].y + _laserCloud->points[i - 3].y
            - 4 * _laserCloud->points[i - 2].y
            + _laserCloud->points[i - 1].y + _laserCloud->points[i].y;

    float ldiffZ =
            _laserCloud->points[i - 4].z + _laserCloud->points[i - 3].z
            - 4 * _laserCloud->points[i - 2].z
            + _laserCloud->points[i - 1].z + _laserCloud->points[i].z;

    float left_curvature = ldiffX * ldiffX + ldiffY * ldiffY + ldiffZ * ldiffZ;

    if(left_curvature < thFlatThreshold * depth){

      std::vector<PointType> left_list;

      for(int j = -4; j < 0; j++){
        left_list.push_back(_laserCloud->points[i + j]);
      }

      left_surf_flag = true;
    }
    else{
      left_surf_flag = false;
    }

    //right curvature
    float rdiffX =
            _laserCloud->points[i + 4].x + _laserCloud->points[i + 3].x
            - 4 * _laserCloud->points[i + 2].x
            + _laserCloud->points[i + 1].x + _laserCloud->points[i].x;

    float rdiffY =
            _laserCloud->points[i + 4].y + _laserCloud->points[i + 3].y
            - 4 * _laserCloud->points[i + 2].y
            + _laserCloud->points[i + 1].y + _laserCloud->points[i].y;

    float rdiffZ =
            _laserCloud->points[i + 4].z + _laserCloud->points[i + 3].z
            - 4 * _laserCloud->points[i + 2].z
            + _laserCloud->points[i + 1].z + _laserCloud->points[i].z;

    float right_curvature = rdiffX * rdiffX + rdiffY * rdiffY + rdiffZ * rdiffZ;

    if(right_curvature < thFlatThreshold * depth){
      std::vector<PointType> right_list;

      for(int j = 1; j < 5; j++){
        right_list.push_back(_laserCloud->points[i + j]);
      }
      count_num = 4;
      right_surf_flag = true;
    }
    else{
      count_num = 1;
      right_surf_flag = false;
    }

    //calculate the included angle
    if(left_surf_flag && right_surf_flag){
      debugnum4 ++;

      Eigen::Vector3d norm_left(0,0,0);
      Eigen::Vector3d norm_right(0,0,0);
      for(int k = 1;k<5;k++){
        Eigen::Vector3d tmp = Eigen::Vector3d(_laserCloud->points[i - k].x - _laserCloud->points[i].x,
                                              _laserCloud->points[i - k].y - _laserCloud->points[i].y,
                                              _laserCloud->points[i - k].z - _laserCloud->points[i].z);
        tmp.normalize();
        norm_left += (k/10.0)* tmp;
      }
      for(int k = 1;k<5;k++){
        Eigen::Vector3d tmp = Eigen::Vector3d(_laserCloud->points[i + k].x - _laserCloud->points[i].x,
                                              _laserCloud->points[i + k].y - _laserCloud->points[i].y,
                                              _laserCloud->points[i + k].z - _laserCloud->points[i].z);
        tmp.normalize();
        norm_right += (k/10.0)* tmp;
      }

      //calculate the angle between this group and the previous group
      double cc = fabs( norm_left.dot(norm_right) / (norm_left.norm()*norm_right.norm()) );
      //calculate the maximum distance, the distance cannot be too small
      Eigen::Vector3d last_tmp = Eigen::Vector3d(_laserCloud->points[i - 4].x - _laserCloud->points[i].x,
                                                 _laserCloud->points[i - 4].y - _laserCloud->points[i].y,
                                                 _laserCloud->points[i - 4].z - _laserCloud->points[i].z);
      Eigen::Vector3d current_tmp = Eigen::Vector3d(_laserCloud->points[i + 4].x - _laserCloud->points[i].x,
                                                    _laserCloud->points[i + 4].y - _laserCloud->points[i].y,
                                                    _laserCloud->points[i + 4].z - _laserCloud->points[i].z);
      double last_dis = last_tmp.norm();
      double current_dis = current_tmp.norm();

      if(cc < 0.5 && last_dis > 0.05 && current_dis > 0.05 ){ //
        debugnum5 ++;
        CloudFeatureFlag[i] = 150;
      }
    }

  }

  //--------------------------------------------------- break points ---------------------------------------------
  for(int i = 5; i < cloudSize - 5; i ++){
    float diff_left[2];
    float diff_right[2];
    float depth = sqrt(_laserCloud->points[i].x * _laserCloud->points[i].x +
                       _laserCloud->points[i].y * _laserCloud->points[i].y +
                       _laserCloud->points[i].z * _laserCloud->points[i].z);

    for(int count = 1; count < 3; count++ ){
      float diffX1 = _laserCloud->points[i + count].x - _laserCloud->points[i].x;
      float diffY1 = _laserCloud->points[i + count].y - _laserCloud->points[i].y;
      float diffZ1 = _laserCloud->points[i + count].z - _laserCloud->points[i].z;
      diff_right[count - 1] = sqrt(diffX1 * diffX1 + diffY1 * diffY1 + diffZ1 * diffZ1);

      float diffX2 = _laserCloud->points[i - count].x - _laserCloud->points[i].x;
      float diffY2 = _laserCloud->points[i - count].y - _laserCloud->points[i].y;
      float diffZ2 = _laserCloud->points[i - count].z - _laserCloud->points[i].z;
      diff_left[count - 1] = sqrt(diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2);
    }

    float depth_right = sqrt(_laserCloud->points[i + 1].x * _laserCloud->points[i + 1].x +
                             _laserCloud->points[i + 1].y * _laserCloud->points[i + 1].y +
                             _laserCloud->points[i + 1].z * _laserCloud->points[i + 1].z);
    float depth_left = sqrt(_laserCloud->points[i - 1].x * _laserCloud->points[i - 1].x +
                            _laserCloud->points[i - 1].y * _laserCloud->points[i - 1].y +
                            _laserCloud->points[i - 1].z * _laserCloud->points[i - 1].z);
    
    if(fabs(diff_right[0] - diff_left[0]) > thBreakCornerDis){
      if(diff_right[0] > diff_left[0]){

        Eigen::Vector3d surf_vector = Eigen::Vector3d(_laserCloud->points[i - 1].x - _laserCloud->points[i].x,
                                                      _laserCloud->points[i - 1].y - _laserCloud->points[i].y,
                                                      _laserCloud->points[i - 1].z - _laserCloud->points[i].z);
        Eigen::Vector3d lidar_vector = Eigen::Vector3d(_laserCloud->points[i].x,
                                                       _laserCloud->points[i].y,
                                                       _laserCloud->points[i].z);
        double left_surf_dis = surf_vector.norm();
        //calculate the angle between the laser direction and the surface
        double cc = fabs( surf_vector.dot(lidar_vector) / (surf_vector.norm()*lidar_vector.norm()) );

        std::vector<PointType> left_list;
        double min_dis = 10000;
        double max_dis = 0;
        for(int j = 0; j < 4; j++){   //TODO: change the plane window size and add thin rod support
          left_list.push_back(_laserCloud->points[i - j]);
          Eigen::Vector3d temp_vector = Eigen::Vector3d(_laserCloud->points[i - j].x - _laserCloud->points[i - j - 1].x,
                                                        _laserCloud->points[i - j].y - _laserCloud->points[i - j - 1].y,
                                                        _laserCloud->points[i - j].z - _laserCloud->points[i - j - 1].z);

          if(j == 3) break;
          double temp_dis = temp_vector.norm();
          if(temp_dis < min_dis) min_dis = temp_dis;
          if(temp_dis > max_dis) max_dis = temp_dis;
        }
        bool left_is_plane = plane_judge(left_list,100);

        if( cc < 0.95 ){//(max_dis < 2*min_dis) && left_surf_dis < 0.05 * depth  && left_is_plane &&
          if(depth_right > depth_left){
            CloudFeatureFlag[i] = 100;
          }
          else{
            if(depth_right == 0) CloudFeatureFlag[i] = 100;
          }
        }
      }
      else{

        Eigen::Vector3d surf_vector = Eigen::Vector3d(_laserCloud->points[i + 1].x - _laserCloud->points[i].x,
                                                      _laserCloud->points[i + 1].y - _laserCloud->points[i].y,
                                                      _laserCloud->points[i + 1].z - _laserCloud->points[i].z);
        Eigen::Vector3d lidar_vector = Eigen::Vector3d(_laserCloud->points[i].x,
                                                       _laserCloud->points[i].y,
                                                       _laserCloud->points[i].z);
        double right_surf_dis = surf_vector.norm();
        //calculate the angle between the laser direction and the surface
        double cc = fabs( surf_vector.dot(lidar_vector) / (surf_vector.norm()*lidar_vector.norm()) );

        std::vector<PointType> right_list;
        double min_dis = 10000;
        double max_dis = 0;
        for(int j = 0; j < 4; j++){ //TODO: change the plane window size and add thin rod support
          right_list.push_back(_laserCloud->points[i - j]);
          Eigen::Vector3d temp_vector = Eigen::Vector3d(_laserCloud->points[i + j].x - _laserCloud->points[i + j + 1].x,
                                                        _laserCloud->points[i + j].y - _laserCloud->points[i + j + 1].y,
                                                        _laserCloud->points[i + j].z - _laserCloud->points[i + j + 1].z);

          if(j == 3) break;
          double temp_dis = temp_vector.norm();
          if(temp_dis < min_dis) min_dis = temp_dis;
          if(temp_dis > max_dis) max_dis = temp_dis;
        }
        bool right_is_plane = plane_judge(right_list,100);

        if( cc < 0.95){ //right_is_plane  && (max_dis < 2*min_dis) && right_surf_dis < 0.05 * depth &&

          if(depth_right < depth_left){
            CloudFeatureFlag[i] = 100;
          }
          else{
            if(depth_left == 0) CloudFeatureFlag[i] = 100;
          }
        }
      }
    }

    // break points select
    if(CloudFeatureFlag[i] == 100){
      debugnum2++;
      std::vector<Eigen::Vector3d> front_norms;
      Eigen::Vector3d norm_front(0,0,0);
      Eigen::Vector3d norm_back(0,0,0);

      for(int k = 1;k<4;k++){

        float temp_depth = sqrt(_laserCloud->points[i - k].x * _laserCloud->points[i - k].x +
                        _laserCloud->points[i - k].y * _laserCloud->points[i - k].y +
                        _laserCloud->points[i - k].z * _laserCloud->points[i - k].z);

        if(temp_depth < 1){
          continue;
        }

        Eigen::Vector3d tmp = Eigen::Vector3d(_laserCloud->points[i - k].x - _laserCloud->points[i].x,
                                              _laserCloud->points[i - k].y - _laserCloud->points[i].y,
                                              _laserCloud->points[i - k].z - _laserCloud->points[i].z);
        tmp.normalize();
        front_norms.push_back(tmp);
        norm_front += (k/6.0)* tmp;
      }
      std::vector<Eigen::Vector3d> back_norms;
      for(int k = 1;k<4;k++){

        float temp_depth = sqrt(_laserCloud->points[i - k].x * _laserCloud->points[i - k].x +
                        _laserCloud->points[i - k].y * _laserCloud->points[i - k].y +
                        _laserCloud->points[i - k].z * _laserCloud->points[i - k].z);

        if(temp_depth < 1){
          continue;
        }

        Eigen::Vector3d tmp = Eigen::Vector3d(_laserCloud->points[i + k].x - _laserCloud->points[i].x,
                                              _laserCloud->points[i + k].y - _laserCloud->points[i].y,
                                              _laserCloud->points[i + k].z - _laserCloud->points[i].z);
        tmp.normalize();
        back_norms.push_back(tmp);
        norm_back += (k/6.0)* tmp;
      }
      double cc = fabs( norm_front.dot(norm_back) / (norm_front.norm()*norm_back.norm()) );
      if(cc < 0.95){
        debugnum3++;
      }else{
        CloudFeatureFlag[i] = 101;
      }

    }

  }

  pcl::PointCloud<PointType>::Ptr laserCloudCorner(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType> cornerPointsSharp;

  std::vector<int> pointsLessSharp_ori;

  int num_surf = 0;
  int num_corner = 0;

  //push_back feature

  for(int i = 5; i < cloudSize - 5; i ++){

    float dis = _laserCloud->points[i].x * _laserCloud->points[i].x
            + _laserCloud->points[i].y * _laserCloud->points[i].y
            + _laserCloud->points[i].z * _laserCloud->points[i].z;

    if(dis < thLidarNearestDis*thLidarNearestDis) continue;

    if(CloudFeatureFlag[i] == 2){
      pointsLessFlat.push_back(i);
      num_surf++;
      continue;
    }

    if(CloudFeatureFlag[i] == 100 || CloudFeatureFlag[i] == 150){ //
      pointsLessSharp_ori.push_back(i);
      laserCloudCorner->push_back(_laserCloud->points[i]);
    }

  }

  for(int i = 0; i < laserCloudCorner->points.size();i++){
      pointsLessSharp.push_back(pointsLessSharp_ori[i]);
      num_corner++;
  }

}

/**
 * @brief 检测CustomMsg的激光雷达特征点,并进行动态障碍物剔除，调用核心函数detectFeaturePoint3()
 * @param[in]  msg 需要检测原始CustomMsg
 * @param[out] laserCloud 将CustomMsg转换为pcl点云格式，进行了0.01近点剔除，其中点的normal法向量存储时间占比，线束信息，特征点分类，1角点，2平面点，3不规则点
 * @param[out] laserConerFeature 从laserCloud中提取的较少Coner特征
 * @param[out] laserSurfFeature 从laserCloud提取的较少平面特征
 * @param[in]  Used_Line 线数
 * @param[in]  lidar_type 雷达类型0-horizon  2-mid360
 */
void LidarFeatureExtractor::FeatureExtract_with_segment(const livox_ros_driver::CustomMsgConstPtr &msg,
                                                        pcl::PointCloud<PointType>::Ptr& laserCloud,
                                                        pcl::PointCloud<PointType>::Ptr& laserConerFeature,
                                                        pcl::PointCloud<PointType>::Ptr& laserSurfFeature,
                                                        pcl::PointCloud<PointType>::Ptr& laserNonFeature,
                                                        sensor_msgs::PointCloud2 &msg_seg,
                                                        const int Used_Line){
  laserCloud->clear();
  laserConerFeature->clear();
  laserSurfFeature->clear();
  laserCloud->clear();
  laserCloud->reserve(15000*N_SCANS); // 避免多次开辟内存
  for(auto & ptr : vlines){
    ptr->clear();
  }
  for(auto & v : vcorner){
    v.clear();
  }
  for(auto & v : vsurf){
    v.clear();
  }

  int dnum = msg->points.size(); // 原始点云点数

  int *idtrans = (int*)calloc(dnum, sizeof(int));
  float *data=(float*)calloc(dnum*4,sizeof(float));
  int point_num = 0;

  double timeSpan = ros::Time().fromNSec(msg->points.back().offset_time).toSec();
  PointType point;
  for(const auto& p : msg->points){
    // 对原始点云的每个点作处理
    int line_num = (int)p.line;
    if(line_num > Used_Line-1) continue; // 超出线数的点剔除
    if(p.x < 0.01) continue; // 0.01近点剔除
    if (!pcl_isfinite(p.x) ||
        !pcl_isfinite(p.y) ||
        !pcl_isfinite(p.z)) {
      continue; // 坐标不正常的点剔除
    }
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.reflectivity;
    point.normal_x = ros::Time().fromNSec(p.offset_time).toSec() /timeSpan;
    point.normal_y = _int_as_float(line_num);
    laserCloud->push_back(point); // 转成PointType存进laserCloud中

    data[point_num*4+0] = point.x;
    data[point_num*4+1] = point.y;
    data[point_num*4+2] = point.z;
    data[point_num*4+3] = point.intensity; // data按顺序存所有点的坐标和强度信息

    point_num++;
  }

  PCSeg pcseg;
  pcseg.DoSeg(idtrans,data,dnum); // 区分地面点、前景点、背景点

  std::size_t cloud_num = laserCloud->size();
  for(std::size_t i=0; i<cloud_num; ++i){
    int line_idx = _float_as_int(laserCloud->points[i].normal_y);
    laserCloud->points[i].normal_z = _int_as_float(i);
    vlines[line_idx]->push_back(laserCloud->points[i]);
  }

  std::thread threads[N_SCANS];
  for(int i=0; i<N_SCANS; ++i){
    threads[i] = std::thread(&LidarFeatureExtractor::detectFeaturePoint3, this, std::ref(vlines[i]),std::ref(vcorner[i]));
  }

  for(int i=0; i<N_SCANS; ++i){
    threads[i].join();
  }

  int num_corner = 0;
  for(int i=0; i<N_SCANS; ++i){
    for(int j=0; j<vcorner[i].size(); ++j){
      laserCloud->points[_float_as_int(vlines[i]->points[vcorner[i][j]].normal_z)].normal_z = 1.0; 
      num_corner++;
    }
  }

  detectFeaturePoint2(laserCloud, laserSurfFeature, laserNonFeature);

  for(std::size_t i=0; i<cloud_num; ++i){
    float dis = laserCloud->points[i].x * laserCloud->points[i].x
                + laserCloud->points[i].y * laserCloud->points[i].y
                + laserCloud->points[i].z * laserCloud->points[i].z;
    if( idtrans[i] > 9 && dis < 50*50){ // 前景
      laserCloud->points[i].normal_z = 0;
    }
  }

  pcl::PointCloud<PointType>::Ptr laserConerFeature_filter;
  laserConerFeature_filter.reset(new pcl::PointCloud<PointType>());
  laserConerFeature.reset(new pcl::PointCloud<PointType>());
  laserSurfFeature.reset(new pcl::PointCloud<PointType>());
  laserNonFeature.reset(new pcl::PointCloud<PointType>());
  for(const auto& p : laserCloud->points){
    if(std::fabs(p.normal_z - 1.0) < 1e-5)
      laserConerFeature->push_back(p);
  }

  for(const auto& p : laserCloud->points){
    if(std::fabs(p.normal_z - 2.0) < 1e-5)
      laserSurfFeature->push_back(p);
    if(std::fabs(p.normal_z - 3.0) < 1e-5)
      laserNonFeature->push_back(p);
  }

}

void LidarFeatureExtractor::FeatureExtract_with_segment_hap(const livox_ros_driver::CustomMsgConstPtr &msg,
                                                            pcl::PointCloud<PointType>::Ptr& laserCloud,
                                                            pcl::PointCloud<PointType>::Ptr& laserConerFeature,
                                                            pcl::PointCloud<PointType>::Ptr& laserSurfFeature,
                                                            pcl::PointCloud<PointType>::Ptr& laserNonFeature,
                                                            sensor_msgs::PointCloud2 &msg_seg,
                                                            const int Used_Line){
  laserCloud->clear();
  laserConerFeature->clear();
  laserSurfFeature->clear();
  laserCloud->clear();
  laserCloud->reserve(15000*N_SCANS);
  for(auto & ptr : vlines){
    ptr->clear();
  }
  for(auto & v : vcorner){
    v.clear();
  }
  for(auto & v : vsurf){
    v.clear();
  }

  int dnum = msg->points.size();

  int *idtrans = (int*)calloc(dnum, sizeof(int));
  float *data=(float*)calloc(dnum*4,sizeof(float));
  int point_num = 0;

  double timeSpan = ros::Time().fromNSec(msg->points.back().offset_time).toSec();
  PointType point;
  for(const auto& p : msg->points){

    int line_num = (int)p.line;
    if(line_num > Used_Line-1) continue;
    if(p.x < 0.01) continue;
    if (!pcl_isfinite(p.x) ||
        !pcl_isfinite(p.y) ||
        !pcl_isfinite(p.z)) {
      continue;
    }
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.reflectivity;
    point.normal_x = ros::Time().fromNSec(p.offset_time).toSec() /timeSpan;
    point.normal_y = _int_as_float(line_num);
    laserCloud->push_back(point);

    data[point_num*4+0] = point.x;
    data[point_num*4+1] = point.y;
    data[point_num*4+2] = point.z;
    data[point_num*4+3] = point.intensity;


    point_num++;
  }

  PCSeg pcseg;
  pcseg.DoSeg(idtrans,data,dnum);

  std::size_t cloud_num = laserCloud->size();

  detectFeaturePoint2(laserCloud, laserSurfFeature, laserNonFeature);

  for(std::size_t i=0; i<cloud_num; ++i){
    float dis = laserCloud->points[i].x * laserCloud->points[i].x
                + laserCloud->points[i].y * laserCloud->points[i].y
                + laserCloud->points[i].z * laserCloud->points[i].z;
    if( idtrans[i] > 9 && dis < 50*50){
      laserCloud->points[i].normal_z = 0;
    }
  }

  pcl::PointCloud<PointType>::Ptr laserConerFeature_filter;
  laserConerFeature_filter.reset(new pcl::PointCloud<PointType>());
  laserConerFeature.reset(new pcl::PointCloud<PointType>());
  laserSurfFeature.reset(new pcl::PointCloud<PointType>());
  laserNonFeature.reset(new pcl::PointCloud<PointType>());
  for(const auto& p : laserCloud->points){
    if(std::fabs(p.normal_z - 1.0) < 1e-5)
      laserConerFeature->push_back(p);
  }

  for(const auto& p : laserCloud->points){
    if(std::fabs(p.normal_z - 2.0) < 1e-5)
      laserSurfFeature->push_back(p);
    if(std::fabs(p.normal_z - 3.0) < 1e-5)
      laserNonFeature->push_back(p);
  }

}


void LidarFeatureExtractor::detectFeaturePoint2(pcl::PointCloud<PointType>::Ptr& cloud,
                                                pcl::PointCloud<PointType>::Ptr& pointsLessFlat,
                                                pcl::PointCloud<PointType>::Ptr& pointsNonFeature){

  int cloudSize = cloud->points.size();

  pointsLessFlat.reset(new pcl::PointCloud<PointType>());
  pointsNonFeature.reset(new pcl::PointCloud<PointType>());

  pcl::KdTreeFLANN<PointType>::Ptr KdTreeCloud;
  KdTreeCloud.reset(new pcl::KdTreeFLANN<PointType>);
  KdTreeCloud->setInputCloud(cloud);

  std::vector<int> _pointSearchInd;
  std::vector<float> _pointSearchSqDis;

  int num_near = 10;
  int stride = 1;
  int interval = 4;

  for(int i = 5; i < cloudSize - 5; i = i+stride) {
    if(fabs(cloud->points[i].normal_z - 1.0) < 1e-5) {
      continue;
    }

    double thre1d = 0.5;
    double thre2d = 0.8;
    double thre3d = 0.5;
    double thre3d2 = 0.13;

    double disti = sqrt(cloud->points[i].x * cloud->points[i].x + 
                        cloud->points[i].y * cloud->points[i].y + 
                        cloud->points[i].z * cloud->points[i].z);

    if(disti < 30.0) {
      thre1d = 0.5;
      thre2d = 0.8;
      thre3d2 = 0.07;
      stride = 14;
      interval = 4;
    } else if(disti < 60.0) {
      stride = 10;
      interval = 3;
    } else {
      stride = 1;
      interval = 0;
    }

    if(disti > 100.0) {
      num_near = 6;

      cloud->points[i].normal_z = 3.0;
      pointsNonFeature->points.push_back(cloud->points[i]);
      continue;
    } else if(disti > 60.0) {
      num_near = 8;
    } else {
      num_near = 10;
    }

    KdTreeCloud->nearestKSearch(cloud->points[i], num_near, _pointSearchInd, _pointSearchSqDis);

    if (_pointSearchSqDis[num_near-1] > 5.0 && disti < 90.0) {
      continue;
    }

    Eigen::Matrix< double, 3, 3 > _matA1;
    _matA1.setZero();

    float cx = 0;
    float cy = 0;
    float cz = 0;
    for (int j = 0; j < num_near; j++) {
      cx += cloud->points[_pointSearchInd[j]].x;
      cy += cloud->points[_pointSearchInd[j]].y;
      cz += cloud->points[_pointSearchInd[j]].z;
    }
    cx /= num_near;
    cy /= num_near;
    cz /= num_near;

    float a11 = 0;
    float a12 = 0;
    float a13 = 0;
    float a22 = 0;
    float a23 = 0;
    float a33 = 0;
    for (int j = 0; j < num_near; j++) {
      float ax = cloud->points[_pointSearchInd[j]].x - cx;
      float ay = cloud->points[_pointSearchInd[j]].y - cy;
      float az = cloud->points[_pointSearchInd[j]].z - cz;

      a11 += ax * ax;
      a12 += ax * ay;
      a13 += ax * az;
      a22 += ay * ay;
      a23 += ay * az;
      a33 += az * az;
    }
    a11 /= num_near;
    a12 /= num_near;
    a13 /= num_near;
    a22 /= num_near;
    a23 /= num_near;
    a33 /= num_near;

    _matA1(0, 0) = a11;
    _matA1(0, 1) = a12;
    _matA1(0, 2) = a13;
    _matA1(1, 0) = a12;
    _matA1(1, 1) = a22;
    _matA1(1, 2) = a23;
    _matA1(2, 0) = a13;
    _matA1(2, 1) = a23;
    _matA1(2, 2) = a33;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(_matA1);
    double a1d = (sqrt(saes.eigenvalues()[2]) - sqrt(saes.eigenvalues()[1])) / sqrt(saes.eigenvalues()[2]);
    double a2d = (sqrt(saes.eigenvalues()[1]) - sqrt(saes.eigenvalues()[0])) / sqrt(saes.eigenvalues()[2]);
    double a3d = sqrt(saes.eigenvalues()[0]) / sqrt(saes.eigenvalues()[2]);

    if(a2d > thre2d || (a3d < thre3d2 && a1d < thre1d)) {
      for(int k = 1; k < interval; k++) {
        cloud->points[i-k].normal_z = 2.0;
        pointsLessFlat->points.push_back(cloud->points[i-k]);
        cloud->points[i+k].normal_z = 2.0;
        pointsLessFlat->points.push_back(cloud->points[i+k]);
      }
      cloud->points[i].normal_z = 2.0;
      pointsLessFlat->points.push_back(cloud->points[i]);
    } else if(a3d > thre3d) {
      for(int k = 1; k < interval; k++) {
        cloud->points[i-k].normal_z = 3.0;
        pointsNonFeature->points.push_back(cloud->points[i-k]);
        cloud->points[i+k].normal_z = 3.0;
        pointsNonFeature->points.push_back(cloud->points[i+k]);
      }
      cloud->points[i].normal_z = 3.0;
      pointsNonFeature->points.push_back(cloud->points[i]);
    }
  }  
}


void LidarFeatureExtractor::detectFeaturePoint3(pcl::PointCloud<PointType>::Ptr& cloud,
                                                std::vector<int>& pointsLessSharp){
  int CloudFeatureFlag[20000];
  float cloudCurvature[20000];
  float cloudDepth[20000];
  int cloudSortInd[20000];
  float cloudReflect[20000];
  int reflectSortInd[20000];
  int cloudAngle[20000];

  pcl::PointCloud<PointType>::Ptr& laserCloudIn = cloud;

  int cloudSize = laserCloudIn->points.size();

  PointType point;
  pcl::PointCloud<PointType>::Ptr _laserCloud(new pcl::PointCloud<PointType>());
  _laserCloud->reserve(cloudSize);

  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn->points[i].x;
    point.y = laserCloudIn->points[i].y;
    point.z = laserCloudIn->points[i].z;
    point.normal_x = 1.0;
    point.intensity = laserCloudIn->points[i].intensity;

    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    _laserCloud->push_back(point);
    CloudFeatureFlag[i] = 0;
  }

  cloudSize = _laserCloud->size();

  int count_num = 1;
  bool left_surf_flag = false;
  bool right_surf_flag = false;

  //--------------------------------------------------- break points ---------------------------------------------
  for(int i = 5; i < cloudSize - 5; i ++){
    float diff_left[2];
    float diff_right[2];
    float depth = sqrt(_laserCloud->points[i].x * _laserCloud->points[i].x +
                       _laserCloud->points[i].y * _laserCloud->points[i].y +
                       _laserCloud->points[i].z * _laserCloud->points[i].z);

    for(int count = 1; count < 3; count++ ){
      float diffX1 = _laserCloud->points[i + count].x - _laserCloud->points[i].x;
      float diffY1 = _laserCloud->points[i + count].y - _laserCloud->points[i].y;
      float diffZ1 = _laserCloud->points[i + count].z - _laserCloud->points[i].z;
      diff_right[count - 1] = sqrt(diffX1 * diffX1 + diffY1 * diffY1 + diffZ1 * diffZ1);

      float diffX2 = _laserCloud->points[i - count].x - _laserCloud->points[i].x;
      float diffY2 = _laserCloud->points[i - count].y - _laserCloud->points[i].y;
      float diffZ2 = _laserCloud->points[i - count].z - _laserCloud->points[i].z;
      diff_left[count - 1] = sqrt(diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2);
    }

    float depth_right = sqrt(_laserCloud->points[i + 1].x * _laserCloud->points[i + 1].x +
                             _laserCloud->points[i + 1].y * _laserCloud->points[i + 1].y +
                             _laserCloud->points[i + 1].z * _laserCloud->points[i + 1].z);
    float depth_left = sqrt(_laserCloud->points[i - 1].x * _laserCloud->points[i - 1].x +
                            _laserCloud->points[i - 1].y * _laserCloud->points[i - 1].y +
                            _laserCloud->points[i - 1].z * _laserCloud->points[i - 1].z);

    
    if(fabs(diff_right[0] - diff_left[0]) > thBreakCornerDis){
      if(diff_right[0] > diff_left[0]){

        Eigen::Vector3d surf_vector = Eigen::Vector3d(_laserCloud->points[i - 1].x - _laserCloud->points[i].x,
                                                      _laserCloud->points[i - 1].y - _laserCloud->points[i].y,
                                                      _laserCloud->points[i - 1].z - _laserCloud->points[i].z);
        Eigen::Vector3d lidar_vector = Eigen::Vector3d(_laserCloud->points[i].x,
                                                       _laserCloud->points[i].y,
                                                       _laserCloud->points[i].z);
        double left_surf_dis = surf_vector.norm();
        //calculate the angle between the laser direction and the surface
        double cc = fabs( surf_vector.dot(lidar_vector) / (surf_vector.norm()*lidar_vector.norm()) );

        std::vector<PointType> left_list;
        double min_dis = 10000;
        double max_dis = 0;
        for(int j = 0; j < 4; j++){   //TODO: change the plane window size and add thin rod support
          left_list.push_back(_laserCloud->points[i - j]);
          Eigen::Vector3d temp_vector = Eigen::Vector3d(_laserCloud->points[i - j].x - _laserCloud->points[i - j - 1].x,
                                                        _laserCloud->points[i - j].y - _laserCloud->points[i - j - 1].y,
                                                        _laserCloud->points[i - j].z - _laserCloud->points[i - j - 1].z);

          if(j == 3) break;
          double temp_dis = temp_vector.norm();
          if(temp_dis < min_dis) min_dis = temp_dis;
          if(temp_dis > max_dis) max_dis = temp_dis;
        }
        // bool left_is_plane = plane_judge(left_list,0.3);

        if(cc < 0.93){//(max_dis < 2*min_dis) && left_surf_dis < 0.05 * depth  && left_is_plane &&
          if(depth_right > depth_left){
            CloudFeatureFlag[i] = 100;
          }
          else{
            if(depth_right == 0) CloudFeatureFlag[i] = 100;
          }
        }
      }
      else{

        Eigen::Vector3d surf_vector = Eigen::Vector3d(_laserCloud->points[i + 1].x - _laserCloud->points[i].x,
                                                      _laserCloud->points[i + 1].y - _laserCloud->points[i].y,
                                                      _laserCloud->points[i + 1].z - _laserCloud->points[i].z);
        Eigen::Vector3d lidar_vector = Eigen::Vector3d(_laserCloud->points[i].x,
                                                       _laserCloud->points[i].y,
                                                       _laserCloud->points[i].z);
        double right_surf_dis = surf_vector.norm();
        //calculate the angle between the laser direction and the surface
        double cc = fabs( surf_vector.dot(lidar_vector) / (surf_vector.norm()*lidar_vector.norm()) );

        std::vector<PointType> right_list;
        double min_dis = 10000;
        double max_dis = 0;
        for(int j = 0; j < 4; j++){ //TODO: change the plane window size and add thin rod support
          right_list.push_back(_laserCloud->points[i - j]);
          Eigen::Vector3d temp_vector = Eigen::Vector3d(_laserCloud->points[i + j].x - _laserCloud->points[i + j + 1].x,
                                                        _laserCloud->points[i + j].y - _laserCloud->points[i + j + 1].y,
                                                        _laserCloud->points[i + j].z - _laserCloud->points[i + j + 1].z);

          if(j == 3) break;
          double temp_dis = temp_vector.norm();
          if(temp_dis < min_dis) min_dis = temp_dis;
          if(temp_dis > max_dis) max_dis = temp_dis;
        }
        // bool right_is_plane = plane_judge(right_list,0.3);

        if(cc < 0.93){ //right_is_plane  && (max_dis < 2*min_dis) && right_surf_dis < 0.05 * depth &&

          if(depth_right < depth_left){
            CloudFeatureFlag[i] = 100;
          }
          else{
            if(depth_left == 0) CloudFeatureFlag[i] = 100;
          }
        }
      }
    }

    // break points select
    if(CloudFeatureFlag[i] == 100){
      std::vector<Eigen::Vector3d> front_norms;
      Eigen::Vector3d norm_front(0,0,0);
      Eigen::Vector3d norm_back(0,0,0);

      for(int k = 1;k<4;k++){

        float temp_depth = sqrt(_laserCloud->points[i - k].x * _laserCloud->points[i - k].x +
                        _laserCloud->points[i - k].y * _laserCloud->points[i - k].y +
                        _laserCloud->points[i - k].z * _laserCloud->points[i - k].z);

        if(temp_depth < 1){
          continue;
        }

        Eigen::Vector3d tmp = Eigen::Vector3d(_laserCloud->points[i - k].x - _laserCloud->points[i].x,
                                              _laserCloud->points[i - k].y - _laserCloud->points[i].y,
                                              _laserCloud->points[i - k].z - _laserCloud->points[i].z);
        tmp.normalize();
        front_norms.push_back(tmp);
        norm_front += (k/6.0)* tmp;
      }
      std::vector<Eigen::Vector3d> back_norms;
      for(int k = 1;k<4;k++){

        float temp_depth = sqrt(_laserCloud->points[i - k].x * _laserCloud->points[i - k].x +
                        _laserCloud->points[i - k].y * _laserCloud->points[i - k].y +
                        _laserCloud->points[i - k].z * _laserCloud->points[i - k].z);

        if(temp_depth < 1){
          continue;
        }

        Eigen::Vector3d tmp = Eigen::Vector3d(_laserCloud->points[i + k].x - _laserCloud->points[i].x,
                                              _laserCloud->points[i + k].y - _laserCloud->points[i].y,
                                              _laserCloud->points[i + k].z - _laserCloud->points[i].z);
        tmp.normalize();
        back_norms.push_back(tmp);
        norm_back += (k/6.0)* tmp;
      }
      double cc = fabs( norm_front.dot(norm_back) / (norm_front.norm()*norm_back.norm()) );
      if(cc < 0.93){
      }else{
        CloudFeatureFlag[i] = 101;
      }

    }

  }

  pcl::PointCloud<PointType>::Ptr laserCloudCorner(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType> cornerPointsSharp;

  std::vector<int> pointsLessSharp_ori;

  int num_surf = 0;
  int num_corner = 0;

  for(int i = 5; i < cloudSize - 5; i ++){
    Eigen::Vector3d left_pt = Eigen::Vector3d(_laserCloud->points[i - 1].x,
                                              _laserCloud->points[i - 1].y,
                                              _laserCloud->points[i - 1].z);
    Eigen::Vector3d right_pt = Eigen::Vector3d(_laserCloud->points[i + 1].x,
                                               _laserCloud->points[i + 1].y,
                                               _laserCloud->points[i + 1].z);

    Eigen::Vector3d cur_pt = Eigen::Vector3d(_laserCloud->points[i].x,
                                             _laserCloud->points[i].y,
                                             _laserCloud->points[i].z);

    float dis = _laserCloud->points[i].x * _laserCloud->points[i].x +
                _laserCloud->points[i].y * _laserCloud->points[i].y +
                _laserCloud->points[i].z * _laserCloud->points[i].z;

    double clr = fabs(left_pt.dot(right_pt) / (left_pt.norm()*right_pt.norm()));
    double cl = fabs(left_pt.dot(cur_pt) / (left_pt.norm()*cur_pt.norm()));
    double cr = fabs(right_pt.dot(cur_pt) / (right_pt.norm()*cur_pt.norm()));

    if(clr < 0.999){
      CloudFeatureFlag[i] = 200;
    }

    if(dis < thLidarNearestDis*thLidarNearestDis) continue;

    if(CloudFeatureFlag[i] == 100 || CloudFeatureFlag[i] == 200){ //
      pointsLessSharp_ori.push_back(i);
      laserCloudCorner->push_back(_laserCloud->points[i]);
    }
  }

  for(int i = 0; i < laserCloudCorner->points.size();i++){
      pointsLessSharp.push_back(pointsLessSharp_ori[i]);
      num_corner++;
  }

}


void LidarFeatureExtractor::FeatureExtract(const livox_ros_driver::CustomMsgConstPtr &msg,
                                           pcl::PointCloud<PointType>::Ptr& laserCloud,
                                           pcl::PointCloud<PointType>::Ptr& laserConerFeature,
                                           pcl::PointCloud<PointType>::Ptr& laserSurfFeature,
                                           const int Used_Line,const int lidar_type){
  laserCloud->clear();
  laserConerFeature->clear();
  laserSurfFeature->clear();
  laserCloud->reserve(15000*N_SCANS);
  for(auto & ptr : vlines){
  ptr->clear();
  }
  for(auto & v : vcorner){
  v.clear();
  }
  for(auto & v : vsurf){
  v.clear();
  }
  double timeSpan = ros::Time().fromNSec(msg->points.back().offset_time).toSec();
  PointType point;
  for(const auto& p : msg->points){
  int line_num = (int)p.line;
  if(line_num > Used_Line-1) continue;
  if(lidar_type == 0||lidar_type == 1)
  {
      if(p.x < 0.01) continue;
  }
  else if(lidar_type == 2)
  {
      if(std::fabs(p.x) < 0.01) continue;
  }
//  if(p.x < 0.01) continue;
  point.x = p.x;
  point.y = p.y;
  point.z = p.z;
  point.intensity = p.reflectivity;
  point.normal_x = ros::Time().fromNSec(p.offset_time).toSec() /timeSpan;
  point.normal_y = _int_as_float(line_num);
  laserCloud->push_back(point);
  }
  std::size_t cloud_num = laserCloud->size();
  for(std::size_t i=0; i<cloud_num; ++i){
  int line_idx = _float_as_int(laserCloud->points[i].normal_y);
  laserCloud->points[i].normal_z = _int_as_float(i);
  vlines[line_idx]->push_back(laserCloud->points[i]);
  laserCloud->points[i].normal_z = 0;
  }
  std::thread threads[N_SCANS];
  for(int i=0; i<N_SCANS; ++i){
  threads[i] = std::thread(&LidarFeatureExtractor::detectFeaturePoint, this, std::ref(vlines[i]),
                     std::ref(vcorner[i]), std::ref(vsurf[i]));
  }
  for(int i=0; i<N_SCANS; ++i){
  threads[i].join();
  }
  for(int i=0; i<N_SCANS; ++i){
  for(int j=0; j<vcorner[i].size(); ++j){
  laserCloud->points[_float_as_int(vlines[i]->points[vcorner[i][j]].normal_z)].normal_z = 1.0;
  }
  for(int j=0; j<vsurf[i].size(); ++j){
  laserCloud->points[_float_as_int(vlines[i]->points[vsurf[i][j]].normal_z)].normal_z = 2.0;
  }
  }

  for(const auto& p : laserCloud->points){
  if(std::fabs(p.normal_z - 1.0) < 1e-5)
  laserConerFeature->push_back(p);
  }
  for(const auto& p : laserCloud->points){
  if(std::fabs(p.normal_z - 2.0) < 1e-5)
  laserSurfFeature->push_back(p);
  }
}

void LidarFeatureExtractor::FeatureExtract_hap(const livox_ros_driver::CustomMsgConstPtr &msg,
                                               pcl::PointCloud<PointType>::Ptr& laserCloud,
                                               pcl::PointCloud<PointType>::Ptr& laserConerFeature,
                                               pcl::PointCloud<PointType>::Ptr& laserSurfFeature,
                                               pcl::PointCloud<PointType>::Ptr& laserNonFeature,
                                               const int Used_Line){
  laserCloud->clear();
  laserConerFeature->clear();
  laserSurfFeature->clear();
  laserCloud->clear();
  laserCloud->reserve(15000*N_SCANS);
  for(auto & ptr : vlines){
    ptr->clear();
  }
  for(auto & v : vcorner){
    v.clear();
  }
  for(auto & v : vsurf){
    v.clear();
  }

  int dnum = msg->points.size();

  double timeSpan = ros::Time().fromNSec(msg->points.back().offset_time).toSec();
  PointType point;
  for(const auto& p : msg->points){

    int line_num = (int)p.line;
    if(line_num > Used_Line-1) continue;
    if(p.x < 0.01) continue;
    if (!pcl_isfinite(p.x) ||
        !pcl_isfinite(p.y) ||
        !pcl_isfinite(p.z)) {
      continue;
    }
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.reflectivity;
    point.normal_x = ros::Time().fromNSec(p.offset_time).toSec() /timeSpan;
    point.normal_y = _int_as_float(line_num);
    laserCloud->push_back(point);
  }

  detectFeaturePoint2(laserCloud, laserSurfFeature, laserNonFeature);

  pcl::PointCloud<PointType>::Ptr laserConerFeature_filter;
  laserConerFeature_filter.reset(new pcl::PointCloud<PointType>());
  laserConerFeature.reset(new pcl::PointCloud<PointType>());
  laserSurfFeature.reset(new pcl::PointCloud<PointType>());
  laserNonFeature.reset(new pcl::PointCloud<PointType>());
  for(const auto& p : laserCloud->points){
    if(std::fabs(p.normal_z - 1.0) < 1e-5)
      laserConerFeature->push_back(p);
  }

  for(const auto& p : laserCloud->points){
    if(std::fabs(p.normal_z - 2.0) < 1e-5)
      laserSurfFeature->push_back(p);
    if(std::fabs(p.normal_z - 3.0) < 1e-5)
      laserNonFeature->push_back(p);
  }
}

void LidarFeatureExtractor::FeatureExtract_Mid(pcl::PointCloud<pcl::PointXYZINormal>::Ptr &msg,
                                           pcl::PointCloud<PointType>::Ptr& laserConerFeature,
                                           pcl::PointCloud<PointType>::Ptr& laserSurfFeature){
    laserConerFeature->clear();
    laserSurfFeature->clear();
    for(auto & ptr : vlines){
        ptr->clear();
    }
    for(auto & v : vcorner){
        v.clear();
    }
    for(auto & v : vsurf){
        v.clear();
    }
    int cloud_num= msg->points.size();
    for(int i=0; i<cloud_num; ++i){
        int line_idx = std::round(msg->points[i].normal_y);
        msg->points[i].normal_z = _int_as_float(i);

        vlines[line_idx]->push_back(msg->points[i]);

        msg->points[i].normal_z = 0;
    }
    std::thread threads[N_SCANS];
    for(int i=0; i<N_SCANS; ++i){
        threads[i] = std::thread(&LidarFeatureExtractor::detectFeaturePoint, this, std::ref(vlines[i]),
                                 std::ref(vcorner[i]), std::ref(vsurf[i]));
    }
    for(int i=0; i<N_SCANS; ++i){
        threads[i].join();
    }
    for(int i=0; i<N_SCANS; ++i){
        for(int j=0; j<vcorner[i].size(); ++j){
            msg->points[_float_as_int(vlines[i]->points[vcorner[i][j]].normal_z)].normal_z = 1.0;
        }
        for(int j=0; j<vsurf[i].size(); ++j){
            msg->points[_float_as_int(vlines[i]->points[vsurf[i][j]].normal_z)].normal_z = 2.0;
        }
    }
    for(const auto& p : msg->points){
        if(std::fabs(p.normal_z - 1.0) < 1e-5)
            laserConerFeature->push_back(p);
    }
    for(const auto& p : msg->points){
        if(std::fabs(p.normal_z - 2.0) < 1e-5)
            laserSurfFeature->push_back(p);
    }
}

// 以下代码来自LIO-Livox/src/segment/segment.cpp
float tmp_gnd_pose[100*6];
int tem_gnd_num = 0;

PCSeg::PCSeg()
{
    this->posFlag=0;
    this->pVImg=(unsigned char*)calloc(DN_SAMPLE_IMG_NX*DN_SAMPLE_IMG_NY*DN_SAMPLE_IMG_NZ,sizeof(unsigned char));
    this->corPoints=NULL;
}
PCSeg::~PCSeg()
{
    if(this->pVImg!=NULL)
    {
        free(this->pVImg);
    }
    if(this->corPoints!=NULL)
    {
        free(this->corPoints);
    }
}

int PCSeg::DoSeg(int *pLabel1, float* fPoints1, int pointNum)
{
    // FeatureExtract_with_segment() 中调用为 pcseg.DoSeg(idtrans,data,dnum); 
    // 1 down sampling
    /*
    #define DN_SAMPLE_IMG_NX 600 //(GND_IMG_NX)
    #define DN_SAMPLE_IMG_NY 200 //(GND_IMG_NY)
    #define DN_SAMPLE_IMG_NZ 100
    #define DN_SAMPLE_IMG_DX 0.4 //(GND_IMG_DX)
    #define DN_SAMPLE_IMG_DY 0.4 //(GND_IMG_DY)
    #define DN_SAMPLE_IMG_DZ 0.2
    #define DN_SAMPLE_IMG_OFFX 40 //(GND_IMG_OFFX)
    #define DN_SAMPLE_IMG_OFFY 40 //(GND_IMG_OFFY)
    #define DN_SAMPLE_IMG_OFFZ 2.5//2.5
    */
    
    float *fPoints2=(float*)calloc(pointNum*4,sizeof(float));
    int *idtrans1=(int*)calloc(pointNum,sizeof(int));
    int *idtrans2=(int*)calloc(pointNum,sizeof(int));
    int pntNum=0; // 用来存没有访问过的点到fPoints2
    if (this->pVImg == NULL)
    {
        this->pVImg=(unsigned char*)calloc(DN_SAMPLE_IMG_NX*DN_SAMPLE_IMG_NY*DN_SAMPLE_IMG_NZ,sizeof(unsigned char));
    }
    memset(pVImg,0,sizeof(unsigned char)*DN_SAMPLE_IMG_NX*DN_SAMPLE_IMG_NY*DN_SAMPLE_IMG_NZ);//600*200*30  
    
    for(int pid=0;pid<pointNum;pid++)
    {
        // 对每个点依次处理，根据点云中每个点的坐标位置(x,y,z)来获得每个点在3D点云中的索引
        int ix=(fPoints1[pid*4]+DN_SAMPLE_IMG_OFFX)/DN_SAMPLE_IMG_DX; //0-240m -> -40-190m
        int iy=(fPoints1[pid*4+1]+DN_SAMPLE_IMG_OFFY)/DN_SAMPLE_IMG_DY; //-40-40m
        int iz=(fPoints1[pid*4+2]+DN_SAMPLE_IMG_OFFZ)/DN_SAMPLE_IMG_DZ; //认为地面为-1.8？ -2.5~17.5

        idtrans1[pid]=-1;
        if((ix>=0)&&(ix<DN_SAMPLE_IMG_NX)&&(iy>=0)&&(iy<DN_SAMPLE_IMG_NY)&&(iz>=0)&&(iz<DN_SAMPLE_IMG_NZ)) //DN_SAMPLE_IMG_OFFX = 0 因此只保留前半块
        {
            idtrans1[pid]=iz*DN_SAMPLE_IMG_NX*DN_SAMPLE_IMG_NY+iy*DN_SAMPLE_IMG_NX+ix; //记录这个点对应的索引
            if(pVImg[idtrans1[pid]]==0)//没有访问过，肯定栅格内会有重复的，所以fPoints2只取第一个
            {
                fPoints2[pntNum*4]=fPoints1[pid*4];
                fPoints2[pntNum*4+1]=fPoints1[pid*4+1];
                fPoints2[pntNum*4+2]=fPoints1[pid*4+2];
                fPoints2[pntNum*4+3]=fPoints1[pid*4+3];

                idtrans2[pntNum]=idtrans1[pid];

                pntNum++;
            }
            pVImg[idtrans1[pid]]=1; //访问过
        }
    }

    //先进行地面校正
    float tmpPos[6];
    tmpPos[0]=-0.15;
    tmpPos[1]=0;
    tmpPos[2]=1;
    tmpPos[3]=0;
    tmpPos[4]=0;
    tmpPos[5]=-2.04;
    // 从fPoints2中找地面点
    GetGndPos(tmpPos,fPoints2,pntNum); //tempPos是更新后的地面搜索点 & 平均法向量 ys
    memcpy(this->gndPos,tmpPos,6*sizeof(float));
    
    this->posFlag=1;//(this->posFlag+1)%SELF_CALI_FRAMES;

    // 3 点云矫正
    this->CorrectPoints(fPoints2,pntNum,this->gndPos);
    if(this->corPoints!=NULL)
        free(this->corPoints);
    this->corPoints=(float*)calloc(pntNum*4,sizeof(float));
    this->corNum=pntNum;
    memcpy(this->corPoints,fPoints2,4*pntNum*sizeof(float));

    // 4 粗略地面分割for地上分割
    int *pLabelGnd=(int*)calloc(pntNum,sizeof(int));
    int gnum=GndSeg(pLabelGnd,fPoints2,pntNum,1.0);

    // 5 地上分割
    int agnum = pntNum-gnum;
    float *fPoints3=(float*)calloc(agnum*4,sizeof(float));
    int *idtrans3=(int*)calloc(agnum,sizeof(int));
    int *pLabel2=(int*)calloc(pntNum,sizeof(int));
    int agcnt=0;//非地面点数量
    for(int ii=0;ii<pntNum;ii++)
    {
        if(pLabelGnd[ii]==0) //上一步地面标记为1，非地面标记为0
        {
            fPoints3[agcnt*4]=fPoints2[ii*4];
            fPoints3[agcnt*4+1]=fPoints2[ii*4+1];
            fPoints3[agcnt*4+2]=fPoints2[ii*4+2];
            pLabel2[ii] = 2;
            idtrans3[agcnt]=ii; //记录fp3在fp2里面的位置
            agcnt++;
        }
        else
        {
            pLabel2[ii] = 0; //地面为1
        }
        
    }
  
    int *pLabelAg=(int*)calloc(agnum,sizeof(int));
    if (agnum != 0)
    {
        AbvGndSeg(pLabelAg,fPoints3,agnum);  
    }
    else
    {
        std::cout << "0 above ground points!\n";
    }

  
    for(int ii=0;ii<agcnt;ii++)
    {   
        if (pLabelAg[ii] >= 10)//前景为0 背景为1 物体分类后 >=10
            pLabel2[idtrans3[ii]] = 200;//pLabelAg[ii]; //前景
        else if (pLabelAg[ii] > 0)
            pLabel2[idtrans3[ii]] = 1; //背景1 -> 0
        else
        {
            pLabel2[idtrans3[ii]] = -1;
        }
        
    }
     
    for(int pid=0;pid<pntNum;pid++)
    {
        pVImg[idtrans2[pid]] = pLabel2[pid];
    }

    for(int pid = 0; pid < pointNum; pid++)
    {
        if(idtrans1[pid]>=0)
        {
            pLabel1[pid]= pVImg[idtrans1[pid]];
        }
        else
        {
            pLabel1[pid] = -1;//未分类
        }
    }

    free(fPoints2);
    free(idtrans1);
    free(idtrans2);
    free(idtrans3);
    free(fPoints3);
    
    free(pLabelAg);
    free(pLabelGnd);


    free(pLabel2);
}

/*
    /src/segment/pointsCorrect.cpp 
*/
int GetGndPos(float *pos, float *fPoints,int pointNum){
    // 调用 GetGndPos(tmpPos,fPoints2,pntNum);
    float *fPoints3=(float*)calloc(60000*4,sizeof(float));//用来存地面点信息
    int pnum3 = FilterGndForPos_cor(fPoints3,fPoints,pointNum);//地面点数量
    float tmpPos[6];
    if (pnum3 < 3)
    {
        std::cout << "too few ground points!\n";
    }
    int gndnum = CalGndPos_cor(tmpPos,fPoints3,pnum3,1.0);//用法向量判断，获取到法向量 & 地面搜索点，放到tmppos
    if(gnd_pos[5]==0){
        memcpy(gnd_pos,tmpPos,sizeof(tmpPos));
    }
    else{
        if(frame_count<frame_lenth_threshold&&tmpPos[5]!=0){
            if(gndnum>0 && abs(gnd_pos[0]-tmpPos[0])<0.1 && abs(gnd_pos[1]-tmpPos[1])<0.1){//更新法向量            
                for(int i = 0;i<6;i++){
                    gnd_pos[i] = (gnd_pos[i]+tmpPos[i])*0.5;
                }
                frame_count = 0;
            }
            else{
                frame_count++;
            }
        }
        else if(tmpPos[5]!=0){
            memcpy(gnd_pos,tmpPos,sizeof(tmpPos));
            frame_count = 0;
        }
    }
    memcpy(pos,gnd_pos,sizeof(float)*6);
    free(fPoints3);
    return 0;
}

int FilterGndForPos_cor(float* outPoints,float*inPoints,int inNum)
{
    // 调用int pnum3 = FilterGndForPos_cor(fPoints3,fPoints,pointNum);
    int outNum=0;
    float dx=2;
    float dy=2;
    int x_len = 20;
    int y_len = 10;
    int nx=2*x_len/dx; //80
    int ny=2*y_len/dy; //10
    float offx=-20,offy=-10;
    float THR=0.4;
    
    float *imgMinZ=(float*)calloc(nx*ny,sizeof(float));
    float *imgMaxZ=(float*)calloc(nx*ny,sizeof(float));
    float *imgSumZ=(float*)calloc(nx*ny,sizeof(float));
    float *imgMeanZ=(float*)calloc(nx*ny,sizeof(float));
    int *imgNumZ=(int*)calloc(nx*ny,sizeof(int));
    int *idtemp = (int*)calloc(inNum,sizeof(int));
    for(int ii=0;ii<nx*ny;ii++)
    {
        imgMinZ[ii]=10;
        imgMaxZ[ii]=-10;
        imgMeanZ[ii] = -10;
        imgSumZ[ii]=0;
        imgNumZ[ii]=0;
    }

    for(int pid=0;pid<inNum;pid++)
    {
        idtemp[pid] = -1;
        // 获得最低点、最高点、数量、总和
        if((inPoints[pid*4] > -x_len) && (inPoints[pid*4]<x_len) && (inPoints[pid*4+1]>-y_len) && (inPoints[pid*4+1]<y_len)) // 绝对值小于x_len,y_len
        {
            int idx=(inPoints[pid*4]-offx)/dx;
            int idy=(inPoints[pid*4+1]-offy)/dy;
            idtemp[pid] = idx+idy*nx;//获得索引
            if (idtemp[pid] >= nx*ny)
                continue;
            imgSumZ[idx+idy*nx] += inPoints[pid*4+2];
            imgNumZ[idx+idy*nx] +=1;
            if(inPoints[pid*4+2]<imgMinZ[idx+idy*nx])
            {
                imgMinZ[idx+idy*nx]=inPoints[pid*4+2];
            }
            if(inPoints[pid*4+2]>imgMaxZ[idx+idy*nx]){
                imgMaxZ[idx+idy*nx]=inPoints[pid*4+2];
            }
        }
    }
    for(int pid=0;pid<inNum;pid++)
    {
        if (outNum >= 60000)
            break;
        if(idtemp[pid] > 0 && idtemp[pid] < nx*ny)
        {
            imgMeanZ[idtemp[pid]] = float(imgSumZ[idtemp[pid]]/(imgNumZ[idtemp[pid]] + 0.0001));//均值高度
            //最高点与均值高度差小于阈值；点数大于3；均值高度小于1 
            if((imgMaxZ[idtemp[pid]] - imgMeanZ[idtemp[pid]]) < THR && imgNumZ[idtemp[pid]] > 3 && imgMeanZ[idtemp[pid]] < 2)
            {// imgMeanZ[idtemp[pid]]<0&&
                outPoints[outNum*4]=inPoints[pid*4];
                outPoints[outNum*4+1]=inPoints[pid*4+1];
                outPoints[outNum*4+2]=inPoints[pid*4+2];
                outPoints[outNum*4+3]=inPoints[pid*4+3];
                outNum++;
            }
        }
    }
    free(imgMinZ);
    free(imgMaxZ);
    free(imgSumZ);
    free(imgMeanZ);
    free(imgNumZ);
    free(idtemp);
    return outNum;
}

int CalGndPos_cor(float *gnd, float *fPoints,int pointNum,float fSearchRadius)
{
    //调用int gndnum = CalGndPos_cor(tmpPos,fPoints3,pnum3,1.0);
    // 初始化gnd
    for(int ii=0;ii<6;ii++)
    {
        gnd[ii]=0;
    }
    // 转换点云到pcl的格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //去除异常点
    if (pointNum <= 3)
    {
        return 0;
    }
    cloud->width=pointNum;
    cloud->height=1;
    cloud->points.resize(cloud->width*cloud->height);

    for (int pid=0;pid<cloud->points.size();pid++)
    {
        cloud->points[pid].x=fPoints[pid*4];
        cloud->points[pid].y=fPoints[pid*4+1];
        cloud->points[pid].z=fPoints[pid*4+2];
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);
    int nNum=0;
    unsigned char* pLabel = (unsigned char*)calloc(pointNum,sizeof(unsigned char));
    for(int pid=0;pid<pointNum;pid++)
    {
        if ((nNum<1000)&&(pLabel[pid]==0))
        {
            SNeiborPCA_cor npca;
            pcl::PointXYZ searchPoint;
            searchPoint.x=cloud->points[pid].x;
            searchPoint.y=cloud->points[pid].y;
            searchPoint.z=cloud->points[pid].z;

            if(GetNeiborPCA_cor(npca,cloud,kdtree,searchPoint,fSearchRadius)>0)
            {
                for(int ii=0;ii<npca.neibors.size();ii++)
                {
                    pLabel[npca.neibors[ii]]=1;
                }

                if(npca.eigenValuesPCA[1]/(npca.eigenValuesPCA[0] + 0.00001)>5000){ //指的是主方向与次方向差异较大。即这一小块接近平面 sy

                        if(npca.eigenVectorsPCA(2,0)>0) //垂直向上？
                        {
                            gnd[0]+=npca.eigenVectorsPCA(0,0);
                            gnd[1]+=npca.eigenVectorsPCA(1,0);
                            gnd[2]+=npca.eigenVectorsPCA(2,0);

                            gnd[3]+=searchPoint.x;
                            gnd[4]+=searchPoint.y;
                            gnd[5]+=searchPoint.z;
                        }
                        else
                        {
                            gnd[0]+=-npca.eigenVectorsPCA(0,0);
                            gnd[1]+=-npca.eigenVectorsPCA(1,0);
                            gnd[2]+=-npca.eigenVectorsPCA(2,0);

                            gnd[3]+=searchPoint.x;
                            gnd[4]+=searchPoint.y;
                            gnd[5]+=searchPoint.z;
                        }
                        nNum++;

                }
            }
        }
    }
    free(pLabel);
    if(nNum>0)
    {
        for(int ii=0;ii<6;ii++)
        {
            gnd[ii]/=nNum; //平均法向量 & 地面点云的中心
        }
        if(abs(gnd[0])<0.1){
            gnd[0]=gnd[0]*(1-abs(gnd[0]))*4.5;
        }
        else if(abs(gnd[0])<0.2){
            gnd[0]=gnd[0]*(1-abs(gnd[0]))*3.2;
        }
        else{
            gnd[0]=gnd[0]*(1-abs(gnd[0]))*2.8;
        }
        gnd[1] = gnd[1]*2.3;
        
    }
    return nNum;
}

int GetNeiborPCA_cor(SNeiborPCA_cor &npca, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree, pcl::PointXYZ searchPoint, float fSearchRadius)
{
    std::vector<float> k_dis;
    pcl::PointCloud<pcl::PointXYZ>::Ptr subCloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(kdtree.radiusSearch(searchPoint,fSearchRadius,npca.neibors,k_dis)>5)
    {
        subCloud->width=npca.neibors.size();
        subCloud->height=1;
        subCloud->points.resize(subCloud->width*subCloud->height);

        for (int pid=0;pid<subCloud->points.size();pid++)//搜索半径内的地面点云 sy
        {
            subCloud->points[pid].x=cloud->points[npca.neibors[pid]].x;
            subCloud->points[pid].y=cloud->points[npca.neibors[pid]].y;
            subCloud->points[pid].z=cloud->points[npca.neibors[pid]].z;
        }
        //利用PCA主元分析法获得点云的三个主方向，获取质心，计算协方差，获得协方差矩阵，求取协方差矩阵的特征值和特长向量，特征向量即为主方向。 sy
        Eigen::Vector4f pcaCentroid;
    	pcl::compute3DCentroid(*subCloud, pcaCentroid);
	    Eigen::Matrix3f covariance;
	    pcl::computeCovarianceMatrixNormalized(*subCloud, pcaCentroid, covariance);
	    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	    npca.eigenVectorsPCA = eigen_solver.eigenvectors();
	    npca.eigenValuesPCA = eigen_solver.eigenvalues();
        float vsum=npca.eigenValuesPCA(0)+npca.eigenValuesPCA(1)+npca.eigenValuesPCA(2);
        npca.eigenValuesPCA(0)=npca.eigenValuesPCA(0)/(vsum+0.000001);//单位化 sy
        npca.eigenValuesPCA(1)=npca.eigenValuesPCA(1)/(vsum+0.000001);
        npca.eigenValuesPCA(2)=npca.eigenValuesPCA(2)/(vsum+0.000001);
    }
    else
    {
        npca.neibors.clear();
    }
    //std::cout << "in PCA2\n";
    return npca.neibors.size();
}
