// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

# 수학 함수 사용을 위한 헤더 파일 포함
#include <cmath>

# ROS에서 사용하는 Odometry 메시지 헤더 파일 포함
#include <nav_msgs/Odometry.h>

# ROS에서 사용하는 Path 메시지 헤더 파일 포함
#include <nav_msgs/Path.h>

# ROS에서 사용하는 PoseStamped 메시지 헤더 파일 포함
#include <geometry_msgs/PoseStamped.h>

# PCL(Point Cloud Library)에서 포인트 클라우드 관련 헤더 파일 포함
#include <pcl/point_cloud.h>

# PCL에서 포인트 타입 관련 헤더 파일 포함
#include <pcl/point_types.h>

# PCL에서 사용하는 Voxel Grid 필터 헤더 파일 포함 (포인트 클라우드 다운샘플링에 사용)
#include <pcl/filters/voxel_grid.h>

# PCL에서 사용하는 KD-트리(K-Dimensional tree) 관련 헤더 파일 포함 (최근접 이웃 탐색에 사용)
#include <pcl/kdtree/kdtree_flann.h>

# PCL 포인트 클라우드를 ROS 메시지 형식으로 변환하기 위한 헤더 파일 포함
#include <pcl_conversions/pcl_conversions.h>

# ROS의 기본 기능을 사용하기 위한 헤더 파일 포함
#include <ros/ros.h>

// 사실 굉장히 고무적인 거라고 생각함. 
// 원래 loam 알고리즘은, imu를 사용할 생각이였다고 들었다. 
// 지금은 imu를 사용하는 것이 약간은, 표준이 되어버린 지라...
// 2014년에 나온 논문이 이렇게 코드 단에서 생각하고 있었다는 것이, 생각보다 코드 자체에 고민을 많이 넣은 것 같다. 
# ROS에서 사용하는 IMU(Inertial Measurement Unit) 메시지 헤더 파일 포함
#include <sensor_msgs/Imu.h>

# ROS에서 사용하는 PointCloud2 메시지 헤더 파일 포함
#include <sensor_msgs/PointCloud2.h>

# ROS에서 사용하는 Transform 데이터 타입 관련 헤더 파일 포함 (좌표 변환에 사용)
#include <tf/transform_datatypes.h>

# ROS에서 사용하는 Transform Broadcaster 헤더 파일 포함 (좌표 변환 브로드캐스트에 사용)
#include <tf/transform_broadcaster.h>

# Eigen 라이브러리의 행렬 계산 기능을 사용하기 위한 헤더 파일 포함
#include <eigen3/Eigen/Dense>

# 멀티스레딩에서 데이터를 보호하기 위한 뮤텍스 사용을 위한 헤더 파일 포함
#include <mutex>

# FIFO 방식의 데이터 구조인 큐를 사용하기 위한 헤더 파일 포함
#include <queue>

# ALOAM (A-LOAM: Advanced LOAM) 프로젝트에서 사용하는 공통 헤더 파일 포함
#include "aloam_velodyne/common.h"

# ALOAM에서 시간 측정을 위한 TicToc 유틸리티 헤더 파일 포함
#include "aloam_velodyne/tic_toc.h"

# ALOAM에서 Lidar Factor 관련 기능을 사용하기 위한 헤더 파일 포함
#include "lidarFactor.hpp"

# 왜곡 보정 기능을 사용하지 않음을 나타내는 매크로 정의 (0: 비활성화)
#define DISTORTION 0


int corner_correspondence = 0, plane_correspondence = 0; // 코너 대응점과 평면 대응점의 수를 저장하는 변수

constexpr double SCAN_PERIOD = 0.1; // 스캔 주기를 0.1초로 설정
constexpr double DISTANCE_SQ_THRESHOLD = 25; // 거리 제곱 임계값을 25로 설정
constexpr double NEARBY_SCAN = 2.5; // 인접한 스캔 범위를 2.5로 설정

int skipFrameNum = 5; // 프레임을 5개씩 건너뛰기 위한 변수
bool systemInited = false; // 시스템 초기화 여부를 나타내는 변수

double timeCornerPointsSharp = 0; // 뾰족한 코너 포인트의 시간을 저장하는 변수
double timeCornerPointsLessSharp = 0; // 덜 뾰족한 코너 포인트의 시간을 저장하는 변수
double timeSurfPointsFlat = 0; // 평평한 서핑 포인트의 시간을 저장하는 변수
double timeSurfPointsLessFlat = 0; // 덜 평평한 서핑 포인트의 시간을 저장하는 변수
double timeLaserCloudFullRes = 0; // 레이저 클라우드 전체 해상도의 시간을 저장하는 변수

    
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>()); // 마지막 코너 포인트의 k-최근접 이웃(KdTree) 구조체를 생성
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>()); // 마지막 서핑 포인트의 k-최근접 이웃(KdTree) 구조체를 생성

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>()); // 뾰족한 코너 포인트를 저장하는 포인트 클라우드 생성
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>()); // 덜 뾰족한 코너 포인트를 저장하는 포인트 클라우드 생성
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>()); // 평평한 서핑 포인트를 저장하는 포인트 클라우드 생성
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>()); // 덜 평평한 서핑 포인트를 저장하는 포인트 클라우드 생성

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>()); // 마지막 레이저 코너 포인트를 저장하는 포인트 클라우드 생성
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>()); // 마지막 레이저 서핑 포인트를 저장하는 포인트 클라우드 생성
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>()); // 전체 해상도의 레이저 포인트 클라우드 생성

int laserCloudCornerLastNum = 0; // 마지막 코너 포인트의 개수를 저장하는 변수
int laserCloudSurfLastNum = 0; // 마지막 서핑 포인트의 개수를 저장하는 변수

// 현재 프레임에서 월드 프레임으로의 변환
Eigen::Quaterniond q_w_curr(1, 0, 0, 0); // 월드 프레임에서 현재 프레임으로의 회전 변환을 나타내는 쿼터니언
Eigen::Vector3d t_w_curr(0, 0, 0); // 월드 프레임에서 현재 프레임으로의 변위를 나타내는 벡터

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1}; // 이전 프레임에서 현재 프레임으로의 회전 변환을 나타내는 쿼터니언 매개변수
double para_t[3] = {0, 0, 0}; // 이전 프레임에서 현재 프레임으로의 변위를 나타내는 매개변수

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q); // 매개변수 배열을 이용해 쿼터니언으로 매핑
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t); // 매개변수 배열을 이용해 벡터로 매핑

std::queue<sensor_msgs::PointCloud2ConstPtr> cornerSharpBuf; // 뾰족한 코너 포인트의 버퍼 큐 생성
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLessSharpBuf; // 덜 뾰족한 코너 포인트의 버퍼 큐 생성
std::queue<sensor_msgs::PointCloud2ConstPtr> surfFlatBuf; // 평평한 서핑 포인트의 버퍼 큐 생성
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLessFlatBuf; // 덜 평평한 서핑 포인트의 버퍼 큐 생성
std::queue<sensor_msgs::PointCloud2ConstPtr> fullPointsBuf; // 전체 포인트 클라우드의 버퍼 큐 생성
std::mutex mBuf; // 버퍼 접근을 위한 뮤텍스 생성


// LiDAR 포인트의 왜곡을 제거
void TransformToStart(PointType const *const pi, PointType *const po)
{
    // 보간 비율 계산
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;  // 왜곡이 있는 경우 보간 비율 계산
    else
        s = 1.0;  // 왜곡이 없는 경우 비율은 1로 설정

    // 회전 변환 계산 (s만큼 보간된 쿼터니언 사용)
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;  // s만큼 보간된 이동 변환 계산
    Eigen::Vector3d point(pi->x, pi->y, pi->z);  // 입력 포인트의 좌표를 Eigen 벡터로 변환
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;  // 변환된 포인트 계산

    // 변환된 포인트의 좌표를 출력 포인트에 저장
    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;  // 강도 정보 유지
}

// 모든 LiDAR 포인트를 다음 프레임의 시작 위치로 변환
void TransformToEnd(PointType const *const pi, PointType *const po)
{
    // 먼저 포인트의 왜곡을 제거
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    // 왜곡이 제거된 포인트를 Eigen 벡터로 변환
    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);  // 다음 프레임으로 변환

    // 변환된 포인트의 좌표를 출력 포인트에 저장
    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    // 왜곡 시간 정보를 제거
    po->intensity = int(pi->intensity);
}

// 코너 포인트(Sharp)를 처리하는 핸들러
void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    mBuf.lock();  // 버퍼 잠금
    cornerSharpBuf.push(cornerPointsSharp2);  // 버퍼에 포인트 추가
    mBuf.unlock();  // 버퍼 잠금 해제
}

// 덜 날카로운 코너 포인트(Less Sharp)를 처리하는 핸들러
void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    mBuf.lock();  // 버퍼 잠금
    cornerLessSharpBuf.push(cornerPointsLessSharp2);  // 버퍼에 포인트 추가
    mBuf.unlock();  // 버퍼 잠금 해제
}

// 평면 포인트(Flat)를 처리하는 핸들러
void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    mBuf.lock();  // 버퍼 잠금
    surfFlatBuf.push(surfPointsFlat2);  // 버퍼에 포인트 추가
    mBuf.unlock();  // 버퍼 잠금 해제
} 

// 덜 평면 포인트(Less Flat)를 처리하는 핸들러
void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    mBuf.lock();  // 버퍼 잠금
    surfLessFlatBuf.push(surfPointsLessFlat2);  // 버퍼에 포인트 추가
    mBuf.unlock();  // 버퍼 잠금 해제
}

// 모든 포인트 클라우드를 수신하는 핸들러
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    mBuf.lock();  // 버퍼 잠금
    fullPointsBuf.push(laserCloudFullRes2);  // 버퍼에 포인트 추가
    mBuf.unlock();  // 버퍼 잠금 해제
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserOdometry");  // ROS 노드 초기화, 노드 이름은 "laserOdometry"
    ros::NodeHandle nh;  // 노드 핸들 생성

    nh.param<int>("mapping_skip_frame", skipFrameNum, 2);  // 파라미터 서버에서 "mapping_skip_frame" 값을 가져와 skipFrameNum에 저장, 기본값은 2

    printf("Mapping %d Hz \n", 10 / skipFrameNum);  // 매핑 주파수를 출력, 10을 skipFrameNum으로 나눈 값

    // 코너 포인트 (Sharp) 구독자 설정
    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, laserCloudSharpHandler);

    // 덜 날카로운 코너 포인트 (Less Sharp) 구독자 설정
    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);

    // 평면 포인트 (Flat) 구독자 설정
    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, laserCloudFlatHandler);

    // 덜 평면 포인트 (Less Flat) 구독자 설정
    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);

    // 전체 포인트 클라우드 구독자 설정
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, laserCloudFullResHandler);

    // 마지막 코너 포인트 퍼블리셔 설정
    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);

    // 마지막 평면 포인트 퍼블리셔 설정
    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);

    // 전체 포인트 클라우드 퍼블리셔 설정
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);

    // 레이저 오도메트리 퍼블리셔 설정
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);

    // 레이저 경로 퍼블리셔 설정
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

    nav_msgs::Path laserPath;  // 레이저 경로 메시지 객체 생성

    int frameCount = 0;  // 프레임 카운터 초기화
    ros::Rate rate(100);  // 루프 주기 설정 (100Hz)
    while (ros::ok())  // ROS가 실행 중일 동안 루프 실행
    {
        ros::spinOnce();  // 콜백 함수 호출하여 메시지 처리
    
        if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
            !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
            !fullPointsBuf.empty())  // 모든 버퍼가 비어있지 않은 경우
        {
            // 각 버퍼의 타임스탬프 가져오기
            timeCornerPointsSharp = cornerSharpBuf.front()->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf.front()->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf.front()->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf.front()->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf.front()->header.stamp.toSec();
    
            // 모든 타임스탬프가 동기화되지 않은 경우
            if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                timeSurfPointsFlat != timeLaserCloudFullRes ||
                timeSurfPointsLessFlat != timeLaserCloudFullRes)
            {
                printf("unsync message!");  // 동기화되지 않은 메시지 경고 출력
                ROS_BREAK();  // ROS 루프 중단
            }
    
            mBuf.lock();  // 버퍼 잠금
            cornerPointsSharp->clear();  // 코너 포인트 (Sharp) 초기화
            pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);  // ROS 메시지를 PCL 포인트 클라우드로 변환
            cornerSharpBuf.pop();  // 버퍼에서 제거
    
            cornerPointsLessSharp->clear();  // 덜 날카로운 코너 포인트 초기화
            pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
            cornerLessSharpBuf.pop();
    
            surfPointsFlat->clear();  // 평면 포인트 초기화
            pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
            surfFlatBuf.pop();
    
            surfPointsLessFlat->clear();  // 덜 평면 포인트 초기화
            pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
            surfLessFlatBuf.pop();
    
            laserCloudFullRes->clear();  // 전체 포인트 클라우드 초기화
            pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
            fullPointsBuf.pop();
            mBuf.unlock();  // 버퍼 잠금 해제

            
            TicToc t_whole;  // 전체 실행 시간 측정 시작
            // 초기화
            if (!systemInited)
            {
                systemInited = true;  // 시스템 초기화 완료 플래그 설정
                std::cout << "Initialization finished \n";  // 초기화 완료 메시지 출력
            }
            else
            {
                int cornerPointsSharpNum = cornerPointsSharp->points.size();  // 날카로운 코너 포인트의 개수
                int surfPointsFlatNum = surfPointsFlat->points.size();  // 평면 포인트의 개수

                TicToc t_opt;  // 최적화 시간 측정 시작
                for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)  // 최적화를 2회 반복
                {
                    corner_correspondence = 0;  // 코너 대응 개수 초기화
                    plane_correspondence = 0;  // 평면 대응 개수 초기화

                    // ceres::LossFunction *loss_function = NULL;
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);  // 손실 함수로 Huber 손실 사용
                    ceres::LocalParameterization *q_parameterization =
                        new ceres::EigenQuaternionParameterization();  // 쿼터니언 매개변수화 설정
                    ceres::Problem::Options problem_options;  // Ceres 최적화 문제 옵션 설정

                    ceres::Problem problem(problem_options);  // Ceres 최적화 문제 생성
                    problem.AddParameterBlock(para_q, 4, q_parameterization);  // 회전 매개변수 추가
                    problem.AddParameterBlock(para_t, 3);  // 변위 매개변수 추가

                    pcl::PointXYZI pointSel;  // 선택된 포인트
                    std::vector<int> pointSearchInd;  // 포인트 검색 인덱스
                    std::vector<float> pointSearchSqDis;  // 포인트 검색 거리 제곱 값

                    TicToc t_data;  // 데이터 처리 시간 측정 시작
                    // 코너 특징점에 대한 대응점 찾기
                    for (int i = 0; i < cornerPointsSharpNum; ++i)
                    {
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);  // 코너 포인트를 변환하여 시작 위치로 설정
                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);  // KD 트리를 사용하여 가장 가까운 포인트 검색

                        int closestPointInd = -1, minPointInd2 = -1;  // 가장 가까운 포인트 인덱스와 두 번째 인덱스 초기화
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)  // 검색된 포인트가 거리 임계값 이하일 경우
                        {
                            closestPointInd = pointSearchInd[0];  // 가장 가까운 포인트의 인덱스 설정
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);  // 가장 가까운 포인트의 스캔 ID 가져오기

                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;  // 두 번째로 가까운 포인트의 거리 초기화
                            // 스캔 라인의 증가 방향으로 검색
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                            {
                                // 동일한 스캔 라인에 있는 경우 계속
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                    continue;

                                // 인접한 스캔에 있지 않은 경우 루프 종료
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)  // 더 가까운 포인트를 찾은 경우
                                {
                                    minPointSqDis2 = pointSqDis;  // 두 번째로 가까운 포인트의 거리 업데이트
                                    minPointInd2 = j;  // 두 번째로 가까운 포인트의 인덱스 설정
                                }
                            }

                            // 스캔 라인의 감소 방향으로 검색
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // 동일한 스캔 라인에 있는 경우 계속
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                    continue;

                                // 인접한 스캔에 있지 않은 경우 루프 종료
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    // 더 가까운 포인트 찾기
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                        }
                        if (minPointInd2 >= 0) // closestPointInd와 minPointInd2가 모두 유효한 경우
                        {
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                       cornerPointsSharp->points[i].y,
                                                       cornerPointsSharp->points[i].z);  // 현재 포인트
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                         laserCloudCornerLast->points[closestPointInd].y,
                                                         laserCloudCornerLast->points[closestPointInd].z);  // 이전 스캔의 가장 가까운 포인트
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                         laserCloudCornerLast->points[minPointInd2].y,
                                                         laserCloudCornerLast->points[minPointInd2].z);  // 이전 스캔의 두 번째로 가까운 포인트

                            double s;
                            if (DISTORTION)
                                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;  // 왜곡 보정 비율 계산
                            else
                                s = 1.0;  // 왜곡이 없는 경우 비율은 1로 설정
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);  // 코스트 함수 생성
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);  // 잔차 블록 추가
                            corner_correspondence++;  // 코너 대응 개수 증가
                        }
                    }

                     // 스캔 평면 특징점에 대한 대응점 찾기
                    for (int i = 0; i < surfPointsFlatNum; ++i)
                    {
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);  // 평면 포인트를 변환하여 시작 위치로 설정
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);  // KD 트리를 사용하여 가장 가까운 포인트 검색

                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;  // 가장 가까운 포인트 인덱스와 두 번째, 세 번째 인덱스 초기화
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)  // 검색된 포인트가 거리 임계값 이하일 경우
                        {
                            closestPointInd = pointSearchInd[0];  // 가장 가까운 포인트의 인덱스 설정

                            // 가장 가까운 포인트의 스캔 ID 가져오기
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;  // 두 번째와 세 번째로 가까운 포인트의 거리 초기화

                            // 스캔 라인의 증가 방향으로 검색
                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                            {
                                // 인접한 스캔에 있지 않은 경우 루프 종료
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // 동일한 또는 낮은 스캔 라인에 있는 경우
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;  // 두 번째로 가까운 포인트의 거리 업데이트
                                    minPointInd2 = j;  // 두 번째로 가까운 포인트의 인덱스 설정
                                }
                                // 더 높은 스캔 라인에 있는 경우
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;  // 세 번째로 가까운 포인트의 거리 업데이트
                                    minPointInd3 = j;  // 세 번째로 가까운 포인트의 인덱스 설정
                                }
                            }

                            // 스캔 라인의 감소 방향으로 검색
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // 인접한 스캔에 있지 않은 경우 루프 종료
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // 동일하거나 더 높은 스캔 라인에 있는 경우
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;  // 두 번째로 가까운 포인트의 거리 업데이트
                                    minPointInd2 = j;  // 두 번째로 가까운 포인트의 인덱스 설정
                                }
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    // 더 가까운 포인트 찾기
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }


                   if (minPointInd2 >= 0 && minPointInd3 >= 0)  // 두 번째와 세 번째 인덱스가 모두 유효한 경우
                            {

                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                            surfPointsFlat->points[i].y,
                                                            surfPointsFlat->points[i].z);  // 현재 평면 포인트
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                                laserCloudSurfLast->points[closestPointInd].y,
                                                                laserCloudSurfLast->points[closestPointInd].z);  // 가장 가까운 이전 포인트
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                                laserCloudSurfLast->points[minPointInd2].y,
                                                                laserCloudSurfLast->points[minPointInd2].z);  // 두 번째로 가까운 이전 포인트
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                                laserCloudSurfLast->points[minPointInd3].y,
                                                                laserCloudSurfLast->points[minPointInd3].z);  // 세 번째로 가까운 이전 포인트

                                double s;
                                if (DISTORTION)
                                    s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;  // 왜곡 보정 비율 계산
                                else
                                    s = 1.0;  // 왜곡이 없는 경우 비율은 1로 설정
                                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);  // 코스트 함수 생성
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);  // 잔차 블록 추가
                                plane_correspondence++;  // 평면 대응 개수 증가
                            }
                        }
                    }

                    //printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
                    printf("data association time %f ms \n", t_data.toc());  // 데이터 연관 시간 출력

                    if ((corner_correspondence + plane_correspondence) < 10)  // 코너 및 평면 대응 개수가 10보다 적으면 경고 메시지 출력
                    {
                        printf("less correspondence! *************************************************\n");
                    }

                    TicToc t_solver;  // 솔버 시간 측정 시작
                    ceres::Solver::Options options;  // Ceres 솔버 옵션 설정
                    options.linear_solver_type = ceres::DENSE_QR;  // 선형 솔버 유형 설정
                    options.max_num_iterations = 4;  // 최대 반복 횟수 설정
                    options.minimizer_progress_to_stdout = false;  // 최소화 진행 상태 출력 비활성화
                    ceres::Solver::Summary summary;  // 솔버 요약 객체 생성
                    ceres::Solve(options, &problem, &summary);  // 문제 해결
                    printf("solver time %f ms \n", t_solver.toc());  // 솔버 시간 출력
                }
                printf("optimization twice time %f \n", t_opt.toc());  // 최적화 두 번 실행 시간 출력

                t_w_curr = t_w_curr + q_w_curr * t_last_curr;  // 현재 위치 업데이트
                q_w_curr = q_w_curr * q_last_curr;  // 현재 회전 업데이트
            }

            TicToc t_pub;  // 퍼블리싱 시간 측정 시작
         
            // 오도메트리 퍼블리시
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "/camera_init";  // 오도메트리의 기준 프레임 설정
            laserOdometry.child_frame_id = "/laser_odom";  // 오도메트리의 자식 프레임 설정
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);  // 타임스탬프 설정
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();  // 현재 회전 쿼터니언 x 값
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();  // 현재 회전 쿼터니언 y 값
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();  // 현재 회전 쿼터니언 z 값
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();  // 현재 회전 쿼터니언 w 값
            laserOdometry.pose.pose.position.x = t_w_curr.x();  // 현재 위치 x 값
            laserOdometry.pose.pose.position.y = t_w_curr.y();  // 현재 위치 y 값
            laserOdometry.pose.pose.position.z = t_w_curr.z();  // 현재 위치 z 값
            pubLaserOdometry.publish(laserOdometry);  // 레이저 오도메트리 퍼블리시

            geometry_msgs::PoseStamped laserPose;  // 레이저 위치 메시지 생성
            laserPose.header = laserOdometry.header;  // 오도메트리 헤더 복사
            laserPose.pose = laserOdometry.pose.pose;  // 오도메트리 위치 복사
            laserPath.header.stamp = laserOdometry.header.stamp;  // 경로 헤더에 타임스탬프 설정
            laserPath.poses.push_back(laserPose);  // 레이저 위치 추가
            laserPath.header.frame_id = "/camera_init";  // 경로의 기준 프레임 설정
            pubLaserPath.publish(laserPath);  // 레이저 경로 퍼블리시

            // 코너 특징과 평면 특징을 스캔 종료 시점으로 변환
            if (0)
            {
                int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();  // 덜 날카로운 코너 포인트 개수
                for (int i = 0; i < cornerPointsLessSharpNum; i++)
                {
                    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);  // 코너 포인트 변환
                }

                int surfPointsLessFlatNum = surfPointsLessFlat->points.size();  // 덜 평면 포인트 개수
                for (int i = 0; i < surfPointsLessFlatNum; i++)
                {
                    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);  // 평면 포인트 변환
                }

                int laserCloudFullResNum = laserCloudFullRes->points.size();  // 전체 포인트 클라우드 개수
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);  // 전체 포인트 변환
                }
            }

            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;  // 임시 포인트 클라우드에 덜 날카로운 코너 포인트 저장
            cornerPointsLessSharp = laserCloudCornerLast;  // 덜 날카로운 코너 포인트를 마지막 코너 포인트로 교체
            laserCloudCornerLast = laserCloudTemp;  // 마지막 코너 포인트를 임시 포인트 클라우드로 설정

            laserCloudTemp = surfPointsLessFlat;  // 임시 포인트 클라우드에 덜 평면 포인트 저장
            surfPointsLessFlat = laserCloudSurfLast;  // 덜 평면 포인트를 마지막 평면 포인트로 교체
            laserCloudSurfLast = laserCloudTemp;  // 마지막 평면 포인트를 임시 포인트 클라우드로 설정

            laserCloudCornerLastNum = laserCloudCornerLast->points.size();  // 마지막 코너 포인트 개수 설정
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();  // 마지막 평면 포인트 개수 설정

            // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';  // 마지막 코너 및 평면 포인트 개수 출력 (디버그용)

            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);  // KD 트리에 마지막 코너 포인트 설정
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);  // KD 트리에 마지막 평면 포인트 설정
        
        
              if (frameCount % skipFrameNum == 0)  // 프레임 수를 건너뛰기 설정에 따라 퍼블리시 실행
                    {
                        frameCount = 0;  // 프레임 카운터 초기화
        
                        sensor_msgs::PointCloud2 laserCloudCornerLast2;  // 마지막 코너 포인트 클라우드 메시지 생성
                        pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);  // PCL 포인트 클라우드를 ROS 메시지로 변환
                        laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);  // 타임스탬프 설정
                        laserCloudCornerLast2.header.frame_id = "/camera";  // 프레임 ID 설정
                        pubLaserCloudCornerLast.publish(laserCloudCornerLast2);  // 마지막 코너 포인트 퍼블리시
        
                        sensor_msgs::PointCloud2 laserCloudSurfLast2;  // 마지막 평면 포인트 클라우드 메시지 생성
                        pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);  // PCL 포인트 클라우드를 ROS 메시지로 변환
                        laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);  // 타임스탬프 설정
                        laserCloudSurfLast2.header.frame_id = "/camera";  // 프레임 ID 설정
                        pubLaserCloudSurfLast.publish(laserCloudSurfLast2);  // 마지막 평면 포인트 퍼블리시
        
                        sensor_msgs::PointCloud2 laserCloudFullRes3;  // 전체 포인트 클라우드 메시지 생성
                        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);  // PCL 포인트 클라우드를 ROS 메시지로 변환
                        laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);  // 타임스탬프 설정
                        laserCloudFullRes3.header.frame_id = "/camera";  // 프레임 ID 설정
                        pubLaserCloudFullRes.publish(laserCloudFullRes3);  // 전체 포인트 퍼블리시
                    }
                    printf("publication time %f ms \n", t_pub.toc());  // 퍼블리시 시간 출력
                    printf("whole laserOdometry time %f ms \n \n", t_whole.toc());  // 전체 레이저 오도메트리 시간 출력
                    if(t_whole.toc() > 100)  // 오도메트리 처리 시간이 100ms를 초과할 경우 경고 출력
                        ROS_WARN("odometry process over 100ms");
        
                    frameCount++;  // 프레임 카운터 증가
                }
                rate.sleep();  // 주기적으로 대기
            }
            return 0;  // 프로그램 종료
        }
