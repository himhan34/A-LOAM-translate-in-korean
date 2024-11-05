// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

// Translate the code with gpt
#include <iostream>  // 표준 입력 및 출력 스트림 라이브러리
#include <fstream>   // 파일 입출력 스트림 라이브러리
#include <iterator>  // 반복자 라이브러리
#include <string>    // 문자열 처리 라이브러리
#include <vector>    // 동적 배열 라이브러리

#include <opencv2/opencv.hpp>  // OpenCV의 주요 함수 및 클래스 포함
#include <image_transport/image_transport.h>  // ROS에서 이미지 전송을 위한 라이브러리
#include <opencv2/highgui/highgui.hpp>  // OpenCV에서 이미지 시각화 관련 함수 포함

#include <nav_msgs/Odometry.h>  // ROS의 위치 추정 데이터를 위한 메시지 타입
#include <nav_msgs/Path.h>  // ROS에서 경로를 저장 및 시각화할 수 있는 메시지 타입

#include <ros/ros.h>  // ROS 기본 기능을 제공하는 라이브러리
#include <rosbag/bag.h>  // ROS에서 데이터 기록 및 재생을 위한 rosbag 라이브러리

#include <geometry_msgs/PoseStamped.h>  // ROS의 위치 및 자세 정보를 담은 메시지 타입

#include <cv_bridge/cv_bridge.h>  // ROS 이미지 데이터를 OpenCV로 변환하기 위한 브리지 라이브러리
#include <sensor_msgs/image_encodings.h>  // ROS 이미지 데이터의 인코딩 정보를 포함

#include <eigen3/Eigen/Dense>  // 행렬 및 벡터 계산을 위한 Eigen 라이브러리

#include <pcl/point_cloud.h>  // PCL(Point Cloud Library)에서 포인트 클라우드 데이터 구조를 포함
#include <pcl/point_types.h>  // PCL에서 다양한 포인트 타입 정의
#include <pcl_conversions/pcl_conversions.h>  // ROS와 PCL 간 데이터 변환 라이브러리
#include <sensor_msgs/PointCloud2.h>  // ROS 포인트 클라우드 데이터를 위한 메시지 타입

std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    // 바이너리 모드로 LiDAR 데이터 파일을 열기
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    
    // 파일 포인터를 파일의 끝으로 이동하여 파일 크기 계산
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);  // 총 float 요소 개수 계산
    
    // 파일 포인터를 다시 파일의 시작으로 이동
    lidar_data_file.seekg(0, std::ios::beg);

    // num_elements 크기의 벡터 생성하여 LiDAR 데이터를 저장할 버퍼로 사용
    std::vector<float> lidar_data_buffer(num_elements);
    
    // 파일에서 데이터를 읽어와 벡터에 저장 (바이트 단위로 읽기)
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements * sizeof(float));
    
    // 읽은 LiDAR 데이터 벡터 반환
    return lidar_data_buffer;
}


int main(int argc, char** argv)
{
    // ROS 노드를 초기화하고 이름을 "kitti_helper"로 설정
    ros::init(argc, argv, "kitti_helper");
    ros::NodeHandle n("~");  // 노드 핸들을 생성하고 네임스페이스를 현재 노드의 private 영역으로 설정
    
    // 파라미터 변수 선언
    std::string dataset_folder, sequence_number, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);  // 데이터셋 폴더 경로를 파라미터에서 가져옴
    n.getParam("sequence_number", sequence_number);  // 시퀀스 번호를 파라미터에서 가져옴
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
    
    bool to_bag;
    n.getParam("to_bag", to_bag);  // bag 파일로 저장할지 여부를 파라미터에서 가져옴
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);  // bag 파일 경로를 파라미터에서 가져옴
    
    int publish_delay;
    n.getParam("publish_delay", publish_delay);  // 데이터의 publish 딜레이를 파라미터에서 가져옴
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;  // 딜레이 값이 0 이하일 경우 1로 설정

    // LiDAR 포인트 클라우드를 퍼블리시할 ROS 토픽 생성
    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);

    // 이미지 퍼블리셔를 위한 ImageTransport 객체 생성 및 좌우 이미지 퍼블리셔 초기화
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);
    image_transport::Publisher pub_image_right = it.advertise("/image_right", 2);

    // Ground Truth 오도메트리 퍼블리셔 설정
    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5);
    nav_msgs::Odometry odomGT;
    odomGT.header.frame_id = "/camera_init";  // 오도메트리의 기준 프레임 설정
    odomGT.child_frame_id = "/ground_truth";  // Ground Truth 프레임 설정

    // Ground Truth 경로 퍼블리셔 설정
    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5);
    nav_msgs::Path pathGT;
    pathGT.header.frame_id = "/camera_init";  // 경로의 기준 프레임 설정

    // 타임스탬프 파일 경로를 설정하고 파일 열기
    std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);

    // Ground Truth 파일 경로를 설정하고 파일 열기
    std::string ground_truth_path = "results/" + sequence_number + ".txt";
    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);

    rosbag::Bag bag_out;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);  // bag 파일을 열어서 쓰기 모드로 설정
    
    // 좌표 변환을 위한 회전 행렬 설정
    Eigen::Matrix3d R_transform;
    R_transform << 0, 0, 1,  // x축 변환
                   -1, 0, 0, // y축 변환
                   0, -1, 0; // z축 변환
    
    // 회전 행렬을 쿼터니언으로 변환하여 q_transform에 저장
    Eigen::Quaterniond q_transform(R_transform);
    
    std::string line;
    std::size_t line_num = 0;  // 파일의 줄 번호를 저장할 변수 초기화
    
    // 퍼블리시 속도를 제어하는 ros::Rate 객체 생성
    ros::Rate r(10.0 / publish_delay);  // 지정된 publish_delay를 기반으로 주기 설정

while (std::getline(timestamp_file, line) && ros::ok())
{
    // 현재 줄의 타임스탬프를 읽어와 float 형식으로 변환
    float timestamp = stof(line);
    
    // 왼쪽 이미지 경로 생성 (파일명을 6자리로 맞추기 위해 0으로 채움)
    std::stringstream left_image_path, right_image_path;
    left_image_path << dataset_folder << "sequences/" + sequence_number + "/image_0/" 
                    << std::setfill('0') << std::setw(6) << line_num << ".png";
    
    // 왼쪽 이미지를 그레이스케일로 읽기
    cv::Mat left_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);
    
    // 오른쪽 이미지 경로 생성 (파일명을 6자리로 맞추기 위해 0으로 채움)
    right_image_path << dataset_folder << "sequences/" + sequence_number + "/image_1/" 
                     << std::setfill('0') << std::setw(6) << line_num << ".png";
    
    // 오른쪽 이미지를 그레이스케일로 읽기
    cv::Mat right_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);

    // ground truth 파일에서 다음 줄을 읽어 포즈 데이터를 문자열로 가져옴
    std::getline(ground_truth_file, line);
    std::stringstream pose_stream(line);
    std::string s;
    
    // ground truth 포즈를 3x4 행렬로 저장
    Eigen::Matrix<double, 3, 4> gt_pose;
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 4; ++j)
        {
            std::getline(pose_stream, s, ' ');  // 공백을 기준으로 나눠 포즈 값을 하나씩 읽음
            gt_pose(i, j) = stof(s);  // 포즈 값을 float 형식으로 변환하여 행렬에 저장
        }
    }
    
        // ground truth 포즈의 회전 부분을 사용해 쿼터니언 생성
        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
        
        // 좌표 변환 쿼터니언과 ground truth 쿼터니언을 곱하여 변환된 쿼터니언 계산
        Eigen::Quaterniond q = q_transform * q_w_i;
        q.normalize();  // 쿼터니언을 정규화
        
        // ground truth 포즈의 위치 부분을 변환하여 새로운 위치 벡터 t 계산
        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();
        
        // odomGT 메시지에 타임스탬프 설정
        odomGT.header.stamp = ros::Time().fromSec(timestamp);
        
        // 변환된 쿼터니언의 x, y, z, w 값을 odomGT 메시지의 방향 값에 할당
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        
        // 변환된 위치 벡터의 x, y, z 값을 odomGT 메시지의 위치 값에 할당
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        
        // 오도메트리 메시지를 퍼블리시
        pubOdomGT.publish(odomGT);
        
        // 경로 메시지의 새로운 포즈를 생성하고 헤더와 포즈 데이터 할당
        geometry_msgs::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        
        // 경로 메시지에 타임스탬프 업데이트하고 poseGT를 추가하여 경로를 확장
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT);
        
        // 경로 메시지를 퍼블리시
        pubPathGT.publish(pathGT);

    // 라이다 포인트 클라우드를 읽습니다.
    std::stringstream lidar_data_path; 
    // 라이다 데이터 경로를 저장할 문자열 스트림을 생성합니다.
    
    lidar_data_path << dataset_folder << "velodyne/sequences/" + sequence_number + "/velodyne/" 
                    << std::setfill('0') << std::setw(6) << line_num << ".bin";
    // 데이터셋 폴더와 시퀀스 번호, 파일 번호(line_num)를 이용해 .bin 파일 경로를 만듭니다. 
    // 파일 번호는 6자리로 설정하고, 비어있는 자리는 0으로 채웁니다.
    
    std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
    // lidar_data_path에서 읽은 라이다 데이터를 float 타입의 벡터로 저장합니다.

   // 현재 라이다 프레임에 포함된 포인트 수를 출력합니다 (각 포인트는 4개의 float 값으로 구성).
    std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

   // 3D 좌표로 이루어진 라이다 포인트들을 저장할 벡터를 생성합니다.
    std::vector<Eigen::Vector3d> lidar_points;

    // 라이다 포인트의 강도(intensity) 값을 저장할 벡터를 생성합니다.
    std::vector<float> lidar_intensities;
    
    // 강도 값을 포함한 XYZ 좌표 포인트 클라우드를 저장할 PCL 포인트 클라우드 객체를 생성합니다.
    pcl::PointCloud<pcl::PointXYZI> laser_cloud;

        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            // LiDAR 데이터의 4개 요소마다 (x, y, z, intensity) 포인트를 생성하여 저장
            lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
            lidar_intensities.push_back(lidar_data[i+3]);

            // 포인트 클라우드 메시지를 위해 pcl::PointXYZI 포인트를 생성하고, 각 요소를 할당
            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laser_cloud.push_back(point); // 포인트 클라우드에 추가
        }

        // 포인트 클라우드 데이터를 ROS 메시지로 변환
        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
    
        // 메시지의 타임스탬프 설정
        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        // 메시지의 프레임 ID 설정 (기준 좌표계 설정)
        laser_cloud_msg.header.frame_id = "/camera_init";
    
        // laser_cloud 메시지를 퍼블리시
        pub_laser_cloud.publish(laser_cloud_msg);

        // CvImage를 사용하여 왼쪽 이미지를 sensor_msgs::ImagePtr 형식으로 변환하고 퍼블리시
        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();
        // CvImage를 사용하여 오른쪽 이미지를 sensor_msgs::ImagePtr 형식으로 변환하고 퍼블리시
        sensor_msgs::ImagePtr image_right_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", right_image).toImageMsg();
        // 왼쪽, 오른쪽 이미지 메시지를 퍼블리시
        pub_image_left.publish(image_left_msg);
        pub_image_right.publish(image_right_msg);

        // 데이터를 ROS Bag 파일에 기록하는 조건문
        if (to_bag)
        {
            // 왼쪽, 오른쪽 이미지와 LiDAR 포인트 클라우드 메시지, 그리고 경로 및 위치 데이터를 Bag 파일에 기록
            bag_out.write("/image_left", ros::Time::now(), image_left_msg);
            bag_out.write("/image_right", ros::Time::now(), image_right_msg);
            bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
            bag_out.write("/path_gt", ros::Time::now(), pathGT);
            bag_out.write("/odometry_gt", ros::Time::now(), odomGT);
        }

        line_num++; // 다음 라인을 처리하기 위해 line_num 증가
        r.sleep(); // 루프 주기 동안 휴식 (ROS 루프 속도 유지)
    }
    
    bag_out.close(); // Bag 파일을 닫음
    std::cout << "Done \n"; // 완료 메시지 출력

    return 0; // 프로그램 종료
}

//개인적으로 코드에서 계속 해서 발견되는 이미지에 관한 토픽은 vloam을 위한 준비 같다. 
// liosam을 만질 때도 느낀 loam 좌표계는 생각보다 굉장히 이상하게 복잡해서... 이거는 다들 불만을 표했던 것 중에 하나인 것 같다. 


