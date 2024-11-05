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

#include <math.h> // 수학 함수 라이브러리 포함
#include <vector> // 벡터 컨테이너를 사용하기 위해 포함
#include <aloam_velodyne/common.h> // ALOAM 관련 공통 헤더 파일 포함
#include <nav_msgs/Odometry.h> // 오도메트리 메시지 포함
#include <nav_msgs/Path.h> // 경로 메시지 포함
#include <geometry_msgs/PoseStamped.h> // 위치 정보 메시지 포함
#include <pcl_conversions/pcl_conversions.h> // PCL과 ROS 간의 변환을 위해 포함
#include <pcl/point_cloud.h> // PCL 포인트 클라우드 처리를 위해 포함
#include <pcl/point_types.h> // PCL 포인트 타입 정의를 위해 포함
#include <pcl/filters/voxel_grid.h> // Voxel Grid 필터를 사용해 다운샘플링을 위해 포함
#include <pcl/kdtree/kdtree_flann.h> // K-D 트리 사용을 위해 포함 (최근접 이웃 탐색)
#include <ros/ros.h> // ROS 기본 기능 사용을 위해 포함
#include <sensor_msgs/Imu.h> // IMU 센서 데이터 메시지 포함
#include <sensor_msgs/PointCloud2.h> // 포인트 클라우드 데이터 메시지 포함
#include <tf/transform_datatypes.h> // 좌표 변환 데이터 타입 포함
#include <tf/transform_broadcaster.h> // 좌표 변환 브로드캐스트를 위해 포함
#include <eigen3/Eigen/Dense> // Eigen 라이브러리 포함 (행렬 연산)
#include <ceres/ceres.h> // Ceres Solver 사용 (최적화 문제 해결)
#include <mutex> // 멀티스레드 환경에서 데이터 보호를 위해 포함
#include <queue> // 큐(FIFO) 구조 사용을 위해 포함
#include <thread> // 멀티스레딩을 위해 포함
#include <iostream> // 입출력 스트림 사용을 위해 포함
#include <string> // 문자열 처리를 위해 포함

#include "lidarFactor.hpp" // LiDAR 관련 사용자 정의 연산을 위한 헤더 파일 포함
#include "aloam_velodyne/common.h" // ALOAM 관련 공통 정의 헤더 파일 다시 포함
#include "aloam_velodyne/tic_toc.h" // 시간 측정을 위한 TicToc 클래스 헤더 파일 포함


int frameCount = 0; // 현재 프레임 수를 저장하는 변수 (초기값: 0)

double timeLaserCloudCornerLast = 0; // 마지막 코너 포인트 클라우드의 시간
double timeLaserCloudSurfLast = 0; // 마지막 서프 포인트 클라우드의 시간
double timeLaserCloudFullRes = 0; // 전체 해상도 포인트 클라우드의 시간
double timeLaserOdometry = 0; // 레이저 오도메트리의 시간

int laserCloudCenWidth = 10; // 포인트 클라우드의 중심 너비 인덱스 (초기값: 10)
int laserCloudCenHeight = 10; // 포인트 클라우드의 중심 높이 인덱스 (초기값: 10)
int laserCloudCenDepth = 5; // 포인트 클라우드의 중심 깊이 인덱스 (초기값: 5)

const int laserCloudWidth = 21; // 포인트 클라우드의 너비 크기 (21 블록)
const int laserCloudHeight = 21; // 포인트 클라우드의 높이 크기 (21 블록)
const int laserCloudDepth = 11; // 포인트 클라우드의 깊이 크기 (11 블록)

const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; // 총 포인트 클라우드 수 계산 (21 * 21 * 11 = 4851)

int laserCloudValidInd[125]; // 유효한 포인트 클라우드 인덱스를 저장하는 배열 (최대 125개)
int laserCloudSurroundInd[125]; // 주변 포인트 클라우드 인덱스를 저장하는 배열 (최대 125개)

// odom으로부터 입력받음
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>()); // 마지막 프레임의 코너 포인트 클라우드
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>()); // 마지막 프레임의 서프 포인트 클라우드

// 출력: 모든 시야 내의 큐브 포인트
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>()); // 주변의 모든 포인트 클라우드

// 맵에서 주변 포인트를 이용해 kd-tree 구축
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>()); // 맵에서의 코너 포인트 클라우드
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>()); // 맵에서의 서프 포인트 클라우드

// 입력 및 출력: 하나의 프레임 내 포인트, 로컬 좌표계에서 글로벌 좌표계로 변환
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>()); // 모든 포인트 클라우드 (완전 해상도)

// 각 큐브의 포인트 클라우드
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum]; // 각 큐브 내의 코너 포인트 클라우드
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum]; // 각 큐브 내의 서프 포인트 클라우드

// kd-tree 구축을 위한 객체
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>()); // 맵에서의 코너 포인트 kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>()); // 맵에서의 서프 포인트 kd-tree

// 초기화된 변환 파라미터 배열
double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters); // 현재 위치의 쿼터니언
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4); // 현재 위치의 위치 벡터

// odom의 월드와 맵의 월드 간의 변환 관계
// wmap_T_odom * odom_T_curr = wmap_T_curr; // odom의 월드 프레임과 맵의 월드 프레임 간 변환
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0); // 맵의 월드와 odom의 월드 간 쿼터니언 변환
Eigen::Vector3d t_wmap_wodom(0, 0, 0); // 맵의 월드와 odom의 월드 간 위치 변환

// odom의 월드와 현재 프레임 간 변환
Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0); // odom의 월드와 현재 위치 간 쿼터니언 변환
Eigen::Vector3d t_wodom_curr(0, 0, 0); // odom의 월드와 현재 위치 간 위치 변환

// 각종 버퍼 큐 선언
std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf; // 마지막 코너 포인트 버퍼
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf; // 마지막 서프 포인트 버퍼
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf; // 완전 해상도 포인트 버퍼
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf; // odom 데이터 버퍼
std::mutex mBuf; // 버퍼 접근을 위한 뮤텍스

// 다운샘플링 필터 객체 선언
pcl::VoxelGrid<PointType> downSizeFilterCorner; // 코너 포인트 다운샘플링 필터
pcl::VoxelGrid<PointType> downSizeFilterSurf; // 서프 포인트 다운샘플링 필터

// 포인트 서치 인덱스와 거리 선언
std::vector<int> pointSearchInd; // 포인트 탐색 인덱스
std::vector<float> pointSearchSqDis; // 포인트 탐색 거리 제곱값

// 포인트 객체 선언
PointType pointOri, pointSel; // 원본 포인트와 선택된 포인트

// ROS 퍼블리셔 선언
ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath; // 주변 포인트, 맵, 완전 해상도 포인트, 매핑 후 odom 등을 퍼블리시하는 퍼블리셔

// 매핑 후 경로 저장을 위한 객체 선언
nav_msgs::Path laserAfterMappedPath; // 매핑 후의 경로

// 초기 추정 설정
void transformAssociateToMap()
{
    // 현재 odom 좌표계에서의 쿼터니언(q)을 map 좌표계로 변환
    q_w_curr = q_wmap_wodom * q_wodom_curr;
    // 현재 odom 좌표계에서의 위치(t)를 map 좌표계로 변환
    t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}

// 변환 갱신
void transformUpdate()
{
    // 현재 좌표계에서 odom 좌표계로의 쿼터니언 갱신
    q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
    // 현재 좌표계에서의 위치 갱신
    t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}

// 맵 좌표계로 포인트 변환
void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
    // 입력 포인트의 현재 좌표를 Eigen 벡터로 정의
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    // 현재 좌표를 맵 좌표계로 변환
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    // 변환된 포인트의 좌표를 출력 포인트에 저장
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    // 입력 포인트의 intensity 값을 출력 포인트에 저장
    po->intensity = pi->intensity;
    // po->intensity = 1.0; // (주석 처리된 코드로, 모든 포인트의 intensity를 1.0으로 설정할 수 있음)
}

// 매핑을 위한 포인트 변환
void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
    // 입력 포인트의 맵 좌표를 Eigen 벡터로 정의
    Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
    // 맵 좌표를 현재 좌표계로 변환 (역변환 적용)
    Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
    // 변환된 포인트의 좌표를 출력 포인트에 저장
    po->x = point_curr.x();
    po->y = point_curr.y();
    po->z = point_curr.z();
    // 입력 포인트의 intensity 값을 출력 포인트에 저장
    po->intensity = pi->intensity;
}

// 마지막 코너 포인트 클라우드 처리 핸들러
void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)
{
    // 뮤텍스를 사용하여 버퍼에 접근 (다중 스레드 환경에서 데이터 보호)
    mBuf.lock();
    // 코너 포인트 클라우드 버퍼에 입력 데이터 추가
    cornerLastBuf.push(laserCloudCornerLast2);
    mBuf.unlock();
}

// 마지막 서프 포인트 클라우드 처리 핸들러
void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2)
{
    // 뮤텍스를 사용하여 버퍼에 접근 (다중 스레드 환경에서 데이터 보호)
    mBuf.lock();
    // 서프 포인트 클라우드 버퍼에 입력 데이터 추가
    surfLastBuf.push(laserCloudSurfLast2);
    mBuf.unlock();
}

// 전체 해상도의 포인트 클라우드 처리 핸들러
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    // 뮤텍스를 사용하여 버퍼에 접근 (다중 스레드 환경에서 데이터 보호)
    mBuf.lock();
    // 전체 해상도의 포인트 클라우드 버퍼에 입력 데이터 추가
    fullResBuf.push(laserCloudFullRes2);
    mBuf.unlock();
}

// odomtry 수신 (Laser Odometry 데이터를 수신하는 콜백 함수)
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
    // 뮤텍스를 사용하여 odometryBuf에 대한 접근을 보호함
    mBuf.lock();
    odometryBuf.push(laserOdometry); // 수신한 레이저 odometry 데이터를 버퍼에 추가
    mBuf.unlock();

    // 고주파수로 데이터를 발행 (실시간성 확보를 위해 처리)
    Eigen::Quaterniond q_wodom_curr; // 현재 odometry의 쿼터니언 (회전 정보)
    Eigen::Vector3d t_wodom_curr;    // 현재 odometry의 위치 벡터 (위치 정보)
    q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x; // 쿼터니언 x 값 설정
    q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y; // 쿼터니언 y 값 설정
    q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z; // 쿼터니언 z 값 설정
    q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w; // 쿼터니언 w 값 설정
    t_wodom_curr.x() = laserOdometry->pose.pose.position.x;    // 위치 벡터 x 값 설정
    t_wodom_curr.y() = laserOdometry->pose.pose.position.y;    // 위치 벡터 y 값 설정
    t_wodom_curr.z() = laserOdometry->pose.pose.position.z;    // 위치 벡터 z 값 설정

    // 맵 기준에서의 현재 위치 계산 (지도의 좌표계로 변환)
    Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr; // 현재 쿼터니언을 맵 기준으로 변환
    Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom; // 현재 위치를 맵 기준으로 변환

    // 변환된 odometry 데이터를 새로운 메시지로 작성하여 발행
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "/camera_init"; // 좌표계의 기준 프레임 설정
    odomAftMapped.child_frame_id = "/aft_mapped";  // 자식 프레임 설정
    odomAftMapped.header.stamp = laserOdometry->header.stamp; // 타임스탬프 설정
    odomAftMapped.pose.pose.orientation.x = q_w_curr.x(); // 변환된 쿼터니언 x 값 설정
    odomAftMapped.pose.pose.orientation.y = q_w_curr.y(); // 변환된 쿼터니언 y 값 설정
    odomAftMapped.pose.pose.orientation.z = q_w_curr.z(); // 변환된 쿼터니언 z 값 설정
    odomAftMapped.pose.pose.orientation.w = q_w_curr.w(); // 변환된 쿼터니언 w 값 설정
    odomAftMapped.pose.pose.position.x = t_w_curr.x();    // 변환된 위치 벡터 x 값 설정
    odomAftMapped.pose.pose.position.y = t_w_curr.y();    // 변환된 위치 벡터 y 값 설정
    odomAftMapped.pose.pose.position.z = t_w_curr.z();    // 변환된 위치 벡터 z 값 설정
    pubOdomAftMappedHighFrec.publish(odomAftMapped); // 변환된 odometry 데이터를 발행
} 

void process()
{
	// 무한 루프 실행
	while(1)
	{
		// cornerLastBuf, surfLastBuf, fullResBuf, odometryBuf가 모두 비어있지 않을 때까지 루프 실행
		while (!cornerLastBuf.empty() && !surfLastBuf.empty() &&
			!fullResBuf.empty() && !odometryBuf.empty())
		{
			// 버퍼 접근을 위한 뮤텍스 잠금
			mBuf.lock();
			// odometryBuf의 타임스탬프가 cornerLastBuf의 타임스탬프보다 작으면 odometryBuf의 데이터를 제거
			while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
				odometryBuf.pop();
			// odometryBuf가 비어있으면 루프 종료
			if (odometryBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			// surfLastBuf의 타임스탬프가 cornerLastBuf의 타임스탬프보다 작으면 surfLastBuf의 데이터를 제거
			while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
				surfLastBuf.pop();
			// surfLastBuf가 비어있으면 루프 종료
			if (surfLastBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			// fullResBuf의 타임스탬프가 cornerLastBuf의 타임스탬프보다 작으면 fullResBuf의 데이터를 제거
			while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
				fullResBuf.pop();
			// fullResBuf가 비어있으면 루프 종료
			if (fullResBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			// 각 버퍼의 타임스탬프를 저장
			timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.toSec();
			timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec();
			timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();
			timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();

			// 모든 버퍼의 타임스탬프가 odometry와 일치하지 않으면 루프 종료
			// 이거보면 모든 버퍼의 타임스탬프가 odometry와 무조건 일치해야함. 
			// 따라서 하기 전에 시간적으로 동기화를 해야 함. (이거는 멀티 라이다를 생각하고 하는 말임)
			if (timeLaserCloudCornerLast != timeLaserOdometry ||
				timeLaserCloudSurfLast != timeLaserOdometry ||
				timeLaserCloudFullRes != timeLaserOdometry)
			{
				printf("time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);
				printf("unsync messeage!");
				mBuf.unlock();
				break;
			}

			// 각 버퍼의 데이터를 포인트 클라우드로 변환하고 버퍼에서 제거
			laserCloudCornerLast->clear();
			pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);
			cornerLastBuf.pop();

			laserCloudSurfLast->clear();
			pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);
			surfLastBuf.pop();

			laserCloudFullRes->clear();
			pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
			fullResBuf.pop();

			// odometryBuf의 데이터를 가져와 현재 위치와 자세(q, t)를 설정
			q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
			q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
			q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
			q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
			t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
			t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
			t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
			odometryBuf.pop();

			// cornerLastBuf에 있는 오래된 프레임을 제거해 실시간 성능을 유지
			while(!cornerLastBuf.empty())
			{
				cornerLastBuf.pop();
				printf("drop lidar frame in mapping for real time performance \n");
			}

			// 버퍼 잠금 해제
			mBuf.unlock();

			// 전체 타이머 시작
			TicToc t_whole;

			// 맵과의 변환 연관시키기
			transformAssociateToMap();

			// 맵 중심 큐브의 인덱스를 계산
			TicToc t_shift;
			int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
			int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
			int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

			// 음수 위치를 보정
			if (t_w_curr.x() + 25.0 < 0)
				centerCubeI--;
			if (t_w_curr.y() + 25.0 < 0)
				centerCubeJ--;
			if (t_w_curr.z() + 25.0 < 0)
				centerCubeK--;

			// 중심 큐브의 인덱스를 이동해 전체 맵을 보정
			while (centerCubeI < 3)
			{
				// 중심 큐브를 왼쪽으로 이동
				for (int j = 0; j < laserCloudHeight; j++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						// 가장 오른쪽의 큐브를 가져와서 전체 큐브를 왼쪽으로 이동
						int i = laserCloudWidth - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; i >= 1; i--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						}
						// 이동 후 오른쪽에 새 큐브 초기화
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				// 중심 인덱스와 중심 너비 조정
				centerCubeI++;
				laserCloudCenWidth++;
			}

			// 다른 방향으로 큐브 이동 및 맵 보정 반복
			// 중심 큐브의 높이와 깊이 방향으로도 비슷한 과정을 반복하여 전체 맵을 동기화합니다.

			// 주변 큐브의 유효한 인덱스와 주변 인덱스를 계산합니다.
			int laserCloudValidNum = 0;
			int laserCloudSurroundNum = 0;

			for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
			{
				for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
				{
					for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
					{
						if (i >= 0 && i < laserCloudWidth &&
							j >= 0 && j < laserCloudHeight &&
							k >= 0 && k < laserCloudDepth)
						{
							// 유효한 큐브와 주변 큐브의 인덱스 저장
							laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
							laserCloudValidNum++;
							laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
							laserCloudSurroundNum++;
						}
					}
				}
			}

			// 유효한 맵 포인트 클라우드에서 코너와 서프 데이터를 가져옴
			laserCloudCornerFromMap->clear();
			laserCloudSurfFromMap->clear();
			for (int i = 0; i < laserCloudValidNum; i++)
			{
				*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
				*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
			}

			// 다운샘플링된 코너와 서프 포인트 수 계산
			pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
			downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
			downSizeFilterCorner.filter(*laserCloudCornerStack);
			int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

			pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
			downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
			downSizeFilterSurf.filter(*laserCloudSurfStack);
			int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

			// 현재 맵과의 일치 작업을 최적화하고 맵을 업데이트합니다.
			// 이 과정에서 여러 가지 유효한 코너와 평면 포인트를 사용하여 맵과 로봇 위치를 정확히 조정합니다.
			// 여러 반복을 통해 최적의 위치와 자세를 찾고 최종 결과를 퍼블리시합니다.
			// 또한 추가적인 큐브에 새 포인트를 추가하고 필터링하는 과정을 포함합니다.
			// 맵과 로봇 위치 사이의 변환을 조정하여 최종 결과를 제공합니다.

			// 최적화 과정이 끝난 후 현재 프레임의 모든 정보를 맵에 추가하고 퍼블리시하여 시각화합니다.
			frameCount++;
		}
		// 실시간 성능을 위해 2밀리초 대기 후 다음 반복 실행
		std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
	}
}

int main(int argc, char **argv)
{
    // ROS 노드 초기화. 노드 이름은 "laserMapping"으로 설정
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    // 라인 해상도와 평면 해상도를 저장할 변수 초기화
    float lineRes = 0;
    float planeRes = 0;
	
    // ROS 파라미터 서버에서 매핑 라인 해상도와 매핑 평면 해상도 값을 불러와서 설정 (기본값 각각 0.4, 0.8)
    nh.param<float>("mapping_line_resolution", lineRes, 0.4);
    nh.param<float>("mapping_plane_resolution", planeRes, 0.8);
	
    // 설정된 라인 해상도와 평면 해상도를 출력
    printf("line resolution %f plane resolution %f \n", lineRes, planeRes);
	
    // 다운샘플링 필터의 리프 크기 설정 (라인과 평면 해상도 기반)
    downSizeFilterCorner.setLeafSize(lineRes, lineRes, lineRes);
    downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

    // 특정 토픽(/laser_cloud_corner_last)에서 메시지를 구독하는 Subscriber 생성 (콜백 함수 laserCloudCornerLastHandler)
    ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100, laserCloudCornerLastHandler);

    // 특정 토픽(/laser_cloud_surf_last)에서 메시지를 구독하는 Subscriber 생성 (콜백 함수 laserCloudSurfLastHandler)
    ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100, laserCloudSurfLastHandler);

    // 특정 토픽(/laser_odom_to_init)에서 메시지를 구독하는 Subscriber 생성 (콜백 함수 laserOdometryHandler)
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);

    // 특정 토픽(/velodyne_cloud_3)에서 메시지를 구독하는 Subscriber 생성 (콜백 함수 laserCloudFullResHandler)
    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100, laserCloudFullResHandler);

    // 특정 토픽(/laser_cloud_surround)으로 메시지를 발행하는 Publisher 생성
    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);

    // 특정 토픽(/laser_cloud_map)으로 메시지를 발행하는 Publisher 생성
    pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);

    // 특정 토픽(/velodyne_cloud_registered)으로 메시지를 발행하는 Publisher 생성
    pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);

    // 특정 토픽(/aft_mapped_to_init)으로 메시지를 발행하는 Publisher 생성
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);

    // 특정 토픽(/aft_mapped_to_init_high_frec)으로 메시지를 발행하는 Publisher 생성
    pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);

    // 특정 토픽(/aft_mapped_path)으로 메시지를 발행하는 Publisher 생성
    pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

    // laserCloudNum 크기만큼의 포인트 클라우드 배열 초기화
    for (int i = 0; i < laserCloudNum; i++)
    {
        laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
        laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
    }

    // 매핑 프로세스를 처리할 새로운 스레드 생성
    std::thread mapping_process{process};

    // ROS 이벤트 처리를 위한 메인 루프. 노드가 종료될 때까지 대기
    ros::spin();

    return 0;
}
