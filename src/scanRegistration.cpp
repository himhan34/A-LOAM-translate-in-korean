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

#include <cmath>  // 수학 함수를 사용하기 위한 헤더 파일 포함
#include <vector>  // 벡터 컨테이너를 사용하기 위한 헤더 파일 포함
#include <string>  // 문자열 처리를 위한 헤더 파일 포함
#include "aloam_velodyne/common.h"  // ALOAM 라이브러리의 공통 헤더 파일 포함
#include "aloam_velodyne/tic_toc.h"  // ALOAM의 시간 측정을 위한 tic_toc 헤더 파일 포함
#include <nav_msgs/Odometry.h>  // ROS 네비게이션 메시지 중 위치 정보를 위한 헤더 파일 포함
#include <opencv/cv.h>  // OpenCV 라이브러리 포함 (컴퓨터 비전 처리에 사용)
#include <pcl_conversions/pcl_conversions.h>  // PCL과 ROS 메시지 간의 변환을 위한 헤더 파일 포함
#include <pcl/point_cloud.h>  // PCL의 포인트 클라우드 처리를 위한 헤더 파일 포함
#include <pcl/point_types.h>  // PCL의 포인트 타입 정의를 위한 헤더 파일 포함
#include <pcl/filters/voxel_grid.h>  // PCL의 보셀 그리드 필터를 사용하기 위한 헤더 파일 포함
#include <pcl/kdtree/kdtree_flann.h>  // PCL의 KD-트리 사용을 위한 헤더 파일 포함
#include <ros/ros.h>  // ROS의 기본 기능 사용을 위한 헤더 파일 포함
#include <sensor_msgs/Imu.h>  // ROS IMU 센서 메시지 처리를 위한 헤더 파일 포함
#include <sensor_msgs/PointCloud2.h>  // ROS 포인트 클라우드 메시지 처리를 위한 헤더 파일 포함
#include <tf/transform_datatypes.h>  // TF 변환 데이터 타입을 사용하기 위한 헤더 파일 포함
#include <tf/transform_broadcaster.h>  // TF 변환을 브로드캐스트하기 위한 헤더 파일 포함


using std::atan2;  // atan2 함수를 사용하기 위해 std 네임스페이스에서 가져옴
using std::cos;  // cos 함수를 사용하기 위해 std 네임스페이스에서 가져옴
using std::sin;  // sin 함수를 사용하기 위해 std 네임스페이스에서 가져옴

const double scanPeriod = 0.1;  // 스캔 주기를 0.1초로 설정 (레이저 스캐너의 주기)

const int systemDelay = 0;  // 시스템 초기화 지연 시간을 0으로 설정
int systemInitCount = 0;  // 시스템 초기화 카운터, 초기값은 0
bool systemInited = false;  // 시스템 초기화 여부를 나타내는 플래그, 초기값은 false
int N_SCANS = 0;  // 스캔 개수를 저장하는 변수, 초기값은 0

// 개인적으로 궁금한 거는 400000개의 배열로 만드는 이유가 뭘지가 궁금하다. 
float cloudCurvature[400000];  // 포인트 클라우드 곡률 정보를 저장할 배열
int cloudSortInd[400000];  // 곡률을 기준으로 포인트를 정렬하기 위한 인덱스 배열
int cloudNeighborPicked[400000];  // 이웃 포인트 선택 여부를 저장할 배열
int cloudLabel[400000];  // 포인트의 레이블 정보를 저장할 배열

bool comp (int i, int j) { return (cloudCurvature[i] < cloudCurvature[j]); }  // 곡률 비교 함수, 작은 곡률이 우선하도록 정의

ros::Publisher pubLaserCloud;  // 레이저 클라우드 전체 포인트를 퍼블리시하기 위한 ROS 퍼블리셔
ros::Publisher pubCornerPointsSharp;  // 날카로운 코너 포인트를 퍼블리시하기 위한 ROS 퍼블리셔
ros::Publisher pubCornerPointsLessSharp;  // 덜 날카로운 코너 포인트를 퍼블리시하기 위한 ROS 퍼블리셔
ros::Publisher pubSurfPointsFlat;  // 평탄한 서페이스 포인트를 퍼블리시하기 위한 ROS 퍼블리셔
ros::Publisher pubSurfPointsLessFlat;  // 덜 평탄한 서페이스 포인트를 퍼블리시하기 위한 ROS 퍼블리셔
ros::Publisher pubRemovePoints;  // 제거된 포인트들을 퍼블리시하기 위한 ROS 퍼블리셔
std::vector<ros::Publisher> pubEachScan;  // 각 스캔 데이터를 퍼블리시하기 위한 퍼블리셔 벡터

bool PUB_EACH_LINE = false;  // 각 라인을 퍼블리시할지 여부를 결정하는 플래그, 초기값은 false

double MINIMUM_RANGE = 0.1;  // 유효한 포인트 클라우드의 최소 범위를 0.1m로 설정


template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)  // 입력과 출력이 같은 객체가 아닐 경우
    {
        cloud_out.header = cloud_in.header;  // 출력 포인트 클라우드의 헤더를 입력과 동일하게 설정
        cloud_out.points.resize(cloud_in.points.size());  // 출력 포인트 클라우드의 크기를 입력과 동일하게 초기화
    }

    size_t j = 0;  // 출력 포인트 클라우드의 유효한 포인트 개수를 저장하기 위한 변수

    for (size_t i = 0; i < cloud_in.points.size(); ++i)  // 입력 포인트 클라우드의 모든 포인트를 순회
    {
        // 현재 포인트의 거리 제곱이 임계값의 제곱보다 작으면 해당 포인트는 제거
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        
        cloud_out.points[j] = cloud_in.points[i];  // 유효한 포인트를 출력 포인트 클라우드에 저장
        j++;  // 유효한 포인트 개수 증가
    }
    
    if (j != cloud_in.points.size())  // 유효한 포인트 개수가 입력 포인트 개수와 다를 경우
    {
        cloud_out.points.resize(j);  // 출력 포인트 클라우드의 크기를 유효한 포인트 개수만큼 축소
    }

    cloud_out.height = 1;  // 출력 포인트 클라우드의 높이를 1로 설정 (단일 라인으로 표현)
    cloud_out.width = static_cast<uint32_t>(j);  // 출력 포인트 클라우드의 너비를 유효한 포인트 개수로 설정
    cloud_out.is_dense = true;  // 출력 포인트 클라우드가 밀집되어 있음을 나타내는 플래그 설정
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    if (!systemInited)  // 시스템이 초기화되지 않은 경우
    { 
        systemInitCount++;  // 초기화 카운트 증가
        if (systemInitCount >= systemDelay)  // 초기화 지연 시간이 지났다면
        {
            systemInited = true;  // 시스템 초기화 완료로 설정
        }
        else
            return;  // 아직 초기화가 완료되지 않았으므로 함수 종료
    }

    TicToc t_whole;  // 전체 작업 시간 측정을 위한 타이머 객체 생성
    TicToc t_prepare;  // 준비 단계 시간 측정을 위한 타이머 객체 생성
    std::vector<int> scanStartInd(N_SCANS, 0);  // 각 스캔의 시작 인덱스를 저장할 벡터 초기화
    std::vector<int> scanEndInd(N_SCANS, 0);  // 각 스캔의 끝 인덱스를 저장할 벡터 초기화

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;  // 입력 포인트 클라우드 생성
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);  // ROS 메시지를 PCL 포인트 클라우드로 변환
    std::vector<int> indices;  // 유효한 인덱스를 저장할 벡터

    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);  // NaN 포인트 제거
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);  // 특정 거리 이하의 포인트 제거

    int cloudSize = laserCloudIn.points.size();  // 포인트 클라우드의 크기 저장
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);  // 시작 지점의 방위각 계산
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;  // 끝 지점의 방위각 계산

    if (endOri - startOri > 3 * M_PI)  // 방위각 차이가 3 * PI보다 큰 경우
    {
        endOri -= 2 * M_PI;  // 2 * PI를 빼서 조정
    }
    else if (endOri - startOri < M_PI)  // 방위각 차이가 PI보다 작은 경우
    {
        endOri += 2 * M_PI;  // 2 * PI를 더해서 조정
    }
    //printf("end Ori %f\n", endOri);  // 끝 방위각 출력 (디버깅용)

    bool halfPassed = false;  // 반을 지나갔는지 여부를 나타내는 플래그
    int count = cloudSize;  // 유효한 포인트 개수를 저장할 변수
    PointType point;  // 포인트 타입 객체 생성
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);  // 각 스캔에 해당하는 포인트 클라우드 벡터 생성
    for (int i = 0; i < cloudSize; i++)  // 포인트 클라우드의 모든 포인트에 대해 반복
    {
        point.x = laserCloudIn.points[i].x;  // 포인트의 x 좌표 설정
        point.y = laserCloudIn.points[i].y;  // 포인트의 y 좌표 설정
        point.z = laserCloudIn.points[i].z;  // 포인트의 z 좌표 설정

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;  // 포인트의 각도 계산
        int scanID = 0;  // 스캔 ID 초기화

        if (N_SCANS == 16)  // 16개의 스캔이 있는 경우
        {
            scanID = int((angle + 15) / 2 + 0.5);  // 스캔 ID 계산
            if (scanID > (N_SCANS - 1) || scanID < 0)  // 유효하지 않은 스캔 ID인 경우
            {
                count--;  // 유효한 포인트 개수 감소
                continue;  // 현재 포인트는 처리하지 않음
            }
        }
        else if (N_SCANS == 32)  // 32개의 스캔이 있는 경우
        {
            scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);  // 스캔 ID 계산
            if (scanID > (N_SCANS - 1) || scanID < 0)  // 유효하지 않은 스캔 ID인 경우
            {
                count--;  // 유효한 포인트 개수 감소
                continue;  // 현재 포인트는 처리하지 않음
            }
        }
        else if (N_SCANS == 64)  // 64개의 스캔이 있는 경우
        {
            if (angle >= -8.83)  // 각도가 -8.83 이상인 경우
                scanID = int((2 - angle) * 3.0 + 0.5);  // 스캔 ID 계산
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);  // 스캔 ID 계산 (하단 라인)

            // 사용 가능한 각도 범위 내에서만 처리
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;  // 유효한 포인트 개수 감소
                continue;  // 현재 포인트는 처리하지 않음
            }
        }
        else
        {
            printf("wrong scan number\n");  // 잘못된 스캔 수일 경우 오류 메시지 출력
            ROS_BREAK();  // 프로그램 중단
        }
        //printf("angle %f scanID %d \n", angle, scanID);  // 각도와 스캔 ID 출력 (디버깅용)

        float ori = -atan2(point.y, point.x);  // 포인트의 방위각 계산
        if (!halfPassed)  // 아직 절반을 지나지 않은 경우
        {
            if (ori < startOri - M_PI / 2)  // 방위각이 시작 방위각보다 많이 작은 경우
            {
                ori += 2 * M_PI;  // 2 * PI를 더해서 조정
            }
            else if (ori > startOri + M_PI * 3 / 2)  // 방위각이 시작 방위각보다 많이 큰 경우
            {
                ori -= 2 * M_PI;  // 2 * PI를 빼서 조정
            }

            if (ori - startOri > M_PI)  // 방위각 차이가 PI보다 큰 경우
            {
                halfPassed = true;  // 반을 지난 것으로 설정
            }
        }
        else
        {
            ori += 2 * M_PI;  // 이미 반을 지난 경우 방위각에 2 * PI 더함
            if (ori < endOri - M_PI * 3 / 2)  // 방위각이 끝 방위각보다 많이 작은 경우
            {
                ori += 2 * M_PI;  // 2 * PI를 더해서 조정
            }
            else if (ori > endOri + M_PI / 2)  // 방위각이 끝 방위각보다 많이 큰 경우
            {
                ori -= 2 * M_PI;  // 2 * PI를 빼서 조정
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);  // 상대적인 시간 계산
        point.intensity = scanID + scanPeriod * relTime;  // 포인트의 intensity 설정 (스캔 ID + 상대적인 시간)
        laserCloudScans[scanID].push_back(point);  // 해당 스캔에 포인트 추가
    }
    
    cloudSize = count;  // 유효한 포인트 개수로 클라우드 크기 설정
    printf("points size %d \n", cloudSize);  // 포인트 개수 출력

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());  // 최종 포인트 클라우드 생성
    for (int i = 0; i < N_SCANS; i++)  // 각 스캔에 대해 반복
    {
        scanStartInd[i] = laserCloud->size() + 5;  // 스캔 시작 인덱스 설정
        *laserCloud += laserCloudScans[i];  // 포인트 클라우드에 해당 스캔의 포인트 추가
        scanEndInd[i] = laserCloud->size() - 6;  // 스캔 끝 인덱스 설정
    }

    printf("prepare time %f \n", t_prepare.toc());  // 준비 시간 출력

    for (int i = 5; i < cloudSize - 5; i++)  // 포인트 클라우드의 포인트들에 대해 반복
    {
        // 각 좌표의 차이를 계산하여 곡률 계산
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;  // 곡률 저장
        cloudSortInd[i] = i;  // 포인트 인덱스 저장
        cloudNeighborPicked[i] = 0;  // 이웃 포인트 선택 여부 초기화
        cloudLabel[i] = 0;  // 포인트 레이블 초기화
    }

    TicToc t_pts;  // 포인트 처리 시간 측정을 위한 타이머 객체 생성

    pcl::PointCloud<PointType> cornerPointsSharp;  // 날카로운 코너 포인트 저장을 위한 포인트 클라우드 생성
    pcl::PointCloud<PointType> cornerPointsLessSharp;  // 덜 날카로운 코너 포인트 저장을 위한 포인트 클라우드 생성
    pcl::PointCloud<PointType> surfPointsFlat;  // 평탄한 서페이스 포인트 저장을 위한 포인트 클라우드 생성
    pcl::PointCloud<PointType> surfPointsLessFlat;  // 덜 평탄한 서페이스 포인트 저장을 위한 포인트 클라우드 생성

    float t_q_sort = 0;  // 정렬 시간 측정을 위한 변수 초기화
    for (int i = 0; i < N_SCANS; i++)  // 각 스캔에 대해 반복
    {
        if (scanEndInd[i] - scanStartInd[i] < 6)  // 스캔 포인트 수가 6보다 작은 경우
            continue;  // 해당 스캔은 처리하지 않음
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);  // 덜 평탄한 서페이스 포인트 저장을 위한 포인트 클라우드 생성
        for (int j = 0; j < 6; j++)  // 스캔을 6개 구간으로 나누어 처리
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;  // 구간 시작 인덱스 계산
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;  // 구간 끝 인덱스 계산

            TicToc t_tmp;  // 각 구간의 정렬 시간 측정을 위한 타이머 객체 생성
            std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp);  // 곡률을 기준으로 정렬
            t_q_sort += t_tmp.toc();  // 정렬 시간 누적

            int largestPickedNum = 0;  // 가장 큰 곡률을 가진 포인트의 개수 초기화
            for (int k = ep; k >= sp; k--)  // 구간의 끝에서부터 시작하여 반복
            {
                int ind = cloudSortInd[k];  // 현재 포인트의 인덱스

                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)  // 선택되지 않았고 곡률이 특정 값보다 큰 경우
                {
                    largestPickedNum++;  // 가장 큰 곡률 포인트 개수 증가
                    if (largestPickedNum <= 2)  // 가장 큰 곡률을 가진 포인트 2개까지
                    {
                        cloudLabel[ind] = 2;  // 해당 포인트를 코너 포인트로 레이블 지정
                        cornerPointsSharp.push_back(laserCloud->points[ind]);  // 날카로운 코너 포인트에 추가
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);  // 덜 날카로운 코너 포인트에도 추가
                    }
                    else if (largestPickedNum <= 20)  // 가장 큰 곡률을 가진 포인트 20개까지
                    {
                        cloudLabel[ind] = 1;  // 해당 포인트를 덜 날카로운 코너 포인트로 레이블 지정
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);  // 덜 날카로운 코너 포인트에 추가
                    }
                    else
                    {
                        break;  // 20개를 초과하면 반복 종료
                    }

                    cloudNeighborPicked[ind] = 1;  // 해당 포인트는 선택된 것으로 설정

                    for (int l = 1; l <= 5; l++)  // 현재 포인트의 인근 포인트들을 처리
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)  // 거리 차이가 특정 값보다 큰 경우
                        {
                            break;  // 더 이상 이웃으로 선택하지 않음
                        }

                        cloudNeighborPicked[ind + l] = 1;  // 이웃 포인트 선택됨으로 설정
                    }
                    for (int l = -1; l >= -5; l--)  // 현재 포인트의 이전 인근 포인트들을 처리
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)  // 거리 차이가 특정 값보다 큰 경우
                        {
                            break;  // 더 이상 이웃으로 선택하지 않음
                        }

                        cloudNeighborPicked[ind + l] = 1;  // 이웃 포인트 선택됨으로 설정
                    }
                }
            }

            int smallestPickedNum = 0;  // 가장 작은 곡률을 가진 포인트의 개수 초기화
            for (int k = sp; k <= ep; k++)  // 구간의 시작에서부터 반복
            {
                int ind = cloudSortInd[k];  // 현재 포인트의 인덱스

                if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)  // 선택되지 않았고 곡률이 특정 값보다 작은 경우
                {
                    cloudLabel[ind] = -1;  // 해당 포인트를 평탄한 서페이스 포인트로 레이블 지정
                    surfPointsFlat.push_back(laserCloud->points[ind]);  // 평탄한 서페이스 포인트에 추가

                    smallestPickedNum++;  // 가장 작은 곡률 포인트 개수 증가
                    if (smallestPickedNum >= 4)  // 가장 작은 곡률을 가진 포인트 4개까지
                    {
                        break;  // 4개를 초과하면 반복 종료
                    }

                    cloudNeighborPicked[ind] = 1;  // 해당 포인트는 선택된 것으로 설정
                    for (int l = 1; l <= 5; l++)  // 현재 포인트의 인근 포인트들을 처리
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)  // 거리 차이가 특정 값보다 큰 경우
                        {
                            break;  // 더 이상 이웃으로 선택하지 않음
                        }

                        cloudNeighborPicked[ind + l] = 1;  // 이웃 포인트 선택됨으로 설정
                    }
                    for (int l = -1; l >= -5; l--)  // 현재 포인트의 이전 인근 포인트들을 처리
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)  // 거리 차이가 특정 값보다 큰 경우
                        {
                            break;  // 더 이상 이웃으로 선택하지 않음
                        }

                        cloudNeighborPicked[ind + l] = 1;  // 이웃 포인트 선택됨으로 설정
                    }
                }
            }

            for (int k = sp; k <= ep; k++)  // 구간의 모든 포인트에 대해 반복
            {
                if (cloudLabel[k] <= 0)  // 레이블이 0 이하인 포인트 (평탄한 포인트)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);  // 덜 평탄한 서페이스 포인트에 추가
                }
            }
        }

        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;  // 다운샘플링된 덜 평탄한 서페이스 포인트 클라우드 생성
        pcl::VoxelGrid<PointType> downSizeFilter;  // 보셀 그리드 필터 객체 생성
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);  // 입력 포인트 클라우드 설정
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);  // 보셀 크기 설정
        downSizeFilter.filter(surfPointsLessFlatScanDS);  // 필터링하여 다운샘플링 수행

        surfPointsLessFlat += surfPointsLessFlatScanDS;  // 덜 평탄한 surf 포인트에 다운샘플링된 포인트 추가
    }
    printf("sort q time %f \n", t_q_sort); // "q" 정렬에 소요된 시간을 출력
    printf("seperate points time %f \n", t_pts.toc()); // 포인트 분리에 소요된 시간을 출력
    
    sensor_msgs::PointCloud2 laserCloudOutMsg; // sensor_msgs에서 PointCloud2 메시지를 선언
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg); // laserCloud를 ROS 메시지로 변환
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp; // 원래 메시지의 시간 정보를 유지
    laserCloudOutMsg.header.frame_id = "/camera_init"; // 프레임 아이디를 "/camera_init"으로 설정
    pubLaserCloud.publish(laserCloudOutMsg); // laserCloud를 ROS 주제로 발행
    
    sensor_msgs::PointCloud2 cornerPointsSharpMsg; // 선명한 모서리 점들을 담을 메시지 선언
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg); // cornerPointsSharp를 ROS 메시지로 변환
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp; // 원래 메시지의 시간 정보를 유지
    cornerPointsSharpMsg.header.frame_id = "/camera_init"; // 프레임 아이디를 "/camera_init"으로 설정
    pubCornerPointsSharp.publish(cornerPointsSharpMsg); // cornerPointsSharp를 ROS 주제로 발행
    
    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg; // 덜 선명한 모서리 점들을 담을 메시지 선언
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg); // cornerPointsLessSharp를 ROS 메시지로 변환
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp; // 원래 메시지의 시간 정보를 유지
    cornerPointsLessSharpMsg.header.frame_id = "/camera_init"; // 프레임 아이디를 "/camera_init"으로 설정
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg); // cornerPointsLessSharp를 ROS 주제로 발행
    
    sensor_msgs::PointCloud2 surfPointsFlat2; // 평평한 면 점들을 담을 메시지 선언
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2); // surfPointsFlat를 ROS 메시지로 변환
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp; // 원래 메시지의 시간 정보를 유지
    surfPointsFlat2.header.frame_id = "/camera_init"; // 프레임 아이디를 "/camera_init"으로 설정
    pubSurfPointsFlat.publish(surfPointsFlat2); // surfPointsFlat를 ROS 주제로 발행
    
    sensor_msgs::PointCloud2 surfPointsLessFlat2; // 덜 평평한 면 점들을 담을 메시지 선언
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2); // surfPointsLessFlat를 ROS 메시지로 변환
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp; // 원래 메시지의 시간 정보를 유지
    surfPointsLessFlat2.header.frame_id = "/camera_init"; // 프레임 아이디를 "/camera_init"으로 설정
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2); // surfPointsLessFlat를 ROS 주제로 발행
    
    // 각 스캔 데이터를 개별로 발행하는 코드
    if(PUB_EACH_LINE) // 각 스캔을 개별로 발행하는 옵션이 활성화되었을 때
    {
        for(int i = 0; i < N_SCANS; i++) // 각 스캔 반복
        {
            sensor_msgs::PointCloud2 scanMsg; // 각 스캔 데이터를 담을 메시지 선언
            pcl::toROSMsg(laserCloudScans[i], scanMsg); // laserCloudScans의 i번째 스캔을 ROS 메시지로 변환
            scanMsg.header.stamp = laserCloudMsg->header.stamp; // 원래 메시지의 시간 정보를 유지
            scanMsg.header.frame_id = "/camera_init"; // 프레임 아이디를 "/camera_init"으로 설정
            pubEachScan[i].publish(scanMsg); // 변환한 스캔 데이터를 ROS 주제로 발행
        }
    }
    
    printf("scan registration time %f ms *************\n", t_whole.toc()); // 스캔 등록에 걸린 전체 시간을 출력
    if(t_whole.toc() > 100) // 스캔 등록 시간이 100ms 이상 걸렸을 경우
        ROS_WARN("scan registration process over 100ms"); // 경고 메시지 출력: 스캔 등록 과정이 100ms를 초과함
    }

int main(int argc, char **argv)
{
    // ROS 노드를 초기화하고 이름을 "scanRegistration"으로 설정합니다.
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;

    // 파라미터 서버에서 "scan_line"이라는 파라미터를 읽어옵니다. 기본값은 16입니다.
    nh.param<int>("scan_line", N_SCANS, 16);
    
   // 아마 우리가 바꿔야 할 것은 scan 라인을 40으로 해야 한다는 것이다. 
   // 근데 이상한게, 다른 코드들 조금에는 scan line을 4로 하더라...  
  

    // 파라미터 서버에서 "minimum_range"라는 파라미터를 읽어옵니다. 기본값은 0.1입니다.
    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    // 스캔 라인 수를 출력합니다.
    printf("scan line number %d \n", N_SCANS);

    // 스캔 라인 수가 16, 32, 64가 아닌 경우 오류 메시지를 출력하고 프로그램을 종료합니다.
    // 아마 코드에 40을 넣어서 해야 할 것 같다. 
    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }

    // "/velodyne_points" 주제에서 PointCloud2 메시지를 구독하는 Subscriber를 생성합니다.
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);

    // "/velodyne_cloud_2" 주제로 PointCloud2 메시지를 발행하는 Publisher를 생성합니다.
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    // "/laser_cloud_sharp" 주제로 PointCloud2 메시지를 발행하는 Publisher를 생성합니다.
    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    // "/laser_cloud_less_sharp" 주제로 PointCloud2 메시지를 발행하는 Publisher를 생성합니다.
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    // "/laser_cloud_flat" 주제로 PointCloud2 메시지를 발행하는 Publisher를 생성합니다.
    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    // "/laser_cloud_less_flat" 주제로 PointCloud2 메시지를 발행하는 Publisher를 생성합니다.
    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    // "/laser_remove_points" 주제로 PointCloud2 메시지를 발행하는 Publisher를 생성합니다.
    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

    // PUB_EACH_LINE 플래그가 설정된 경우 각 스캔에 대한 Publisher를 생성합니다.
    if(PUB_EACH_LINE)
    {
        // 각 스캔 ID별로 주제를 발행하기 위해 Publisher를 생성하여 pubEachScan 벡터에 추가합니다.
        for(int i = 0; i < N_SCANS; i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    
    // ROS 콜백 함수가 실행되도록 ros::spin()을 호출하여 노드를 유지합니다.
    ros::spin();

    return 0;
}
