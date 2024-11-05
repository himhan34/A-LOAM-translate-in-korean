// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#include <ceres/ceres.h> // Ceres Solver 라이브러리를 포함합니다.
#include <ceres/rotation.h> // 회전 관련 Ceres 라이브러리를 포함합니다.
#include <eigen3/Eigen/Dense> // 선형 대수 라이브러리인 Eigen을 포함합니다.
#include <pcl/point_cloud.h> // 포인트 클라우드 라이브러리를 포함합니다.
#include <pcl/point_types.h> // PCL의 포인트 타입을 정의한 라이브러리를 포함합니다.
#include <pcl/kdtree/kdtree_flann.h> // KD-트리 탐색을 위한 PCL 라이브러리를 포함합니다.
#include <pcl_conversions/pcl_conversions.h> // ROS와 PCL 간 데이터 변환을 위한 라이브러리를 포함합니다.

struct LidarEdgeFactor // LidarEdgeFactor 구조체를 정의합니다.
{
	LidarEdgeFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_,
					Eigen::Vector3d last_point_b_, double s_)
		: curr_point(curr_point_), last_point_a(last_point_a_), last_point_b(last_point_b_), s(s_) {}
	// 생성자: 현재 포인트(curr_point), 이전 두 포인트(last_point_a, last_point_b), 그리고 스케일 파라미터 s를 초기화합니다.

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		// 연산자 오버로딩: 이 구조체를 CostFunction으로 사용할 수 있도록 설정합니다.
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		// 현재 포인트를 템플릿 타입 T로 변환하여 cp에 저장합니다.

		Eigen::Matrix<T, 3, 1> lpa{T(last_point_a.x()), T(last_point_a.y()), T(last_point_a.z())};
		// 이전 포인트 A를 템플릿 타입 T로 변환하여 lpa에 저장합니다.

		Eigen::Matrix<T, 3, 1> lpb{T(last_point_b.x()), T(last_point_b.y()), T(last_point_b.z())};
		// 이전 포인트 B를 템플릿 타입 T로 변환하여 lpb에 저장합니다.

		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		// 쿼터니언을 사용하여 q_last_curr을 생성합니다. q는 회전을 나타냅니다.

		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		// 단위 쿼터니언을 생성합니다.

		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		// slerp를 사용하여 보간된 쿼터니언을 생성합니다.
		// 이거 궁금한 게, 왜 쿼터니언을 보간하는 것인가. 

		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};
		// 스케일 s를 곱하여 변환 벡터 t_last_curr을 생성합니다.
		// 이것도 궁금한게 변환하는 이유가 스케일을 맞추기 위해서라고 보면 되는 건가... 

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;
		// 쿼터니언과 변환 벡터를 적용하여 현재 포인트 cp를 새로운 좌표 lp로 변환합니다.

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		// 변환된 포인트와 이전 두 포인트 간의 벡터곱을 nu로 계산합니다.

		Eigen::Matrix<T, 3, 1> de = lpa - lpb;
		// 이전 두 포인트 간의 차를 de로 계산합니다.

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();
		// 잔차(residual)를 각 성분별로 계산하여 저장합니다.

		return true;
		// 연산이 성공적으로 완료되었음을 반환합니다.
	}


// 이번 기회에 ceres를 잘 사용하는 법에 대해서 제대로 배우면 좋을 것 같음. 
// 어느정도 시간이 남았으니, 이걸 잘 활용하면 좋을 것 같음. 
// https://github.com/LimHyungTae/helloceres => 한국 slam 학계에서, 가장 활발한 활동을 하시는 현재 mit에 계시는 박사님의 repo임.
// 어떻게 ceres를 사용하면 되는지, 잘 나와있음. 
	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
									   const Eigen::Vector3d last_point_b_, const double s_)
	{
		// 팩토리 함수: LidarEdgeFactor 객체를 생성하고 ceres Solver에서 사용할 수 있도록 포인터를 반환합니다.
		return (new ceres::AutoDiffCostFunction<
				LidarEdgeFactor, 3, 4, 3>(
			new LidarEdgeFactor(curr_point_, last_point_a_, last_point_b_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_a, last_point_b; // 현재 포인트와 이전 두 포인트를 저장할 멤버 변수입니다.
	double s; // 스케일 파라미터 s를 저장할 멤버 변수입니다.
};

struct LidarPlaneFactor // LidarPlaneFactor 구조체를 정의합니다.
{
	LidarPlaneFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),
		  last_point_m(last_point_m_), s(s_)
	{
		// 생성자: 현재 포인트, 이전 세 포인트와 스케일 파라미터를 초기화합니다.

		ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		// 이전 두 포인트 벡터의 외적을 계산하여 ljm_norm에 저장합니다.

		ljm_norm.normalize();
		// ljm_norm 벡터를 단위 벡터로 정규화합니다.
	}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		// 연산자 오버로딩: 이 구조체를 Ceres CostFunction으로 사용할 수 있도록 설정합니다.

		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		// 현재 포인트를 템플릿 타입 T로 변환하여 cp에 저장합니다.

		Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
		// 이전 포인트 j를 템플릿 타입 T로 변환하여 lpj에 저장합니다.

		Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};
		// ljm_norm 벡터를 템플릿 타입 T로 변환하여 ljm에 저장합니다.

		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		// 쿼터니언을 사용하여 q_last_curr을 생성합니다. q는 회전을 나타냅니다.

		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		// 단위 쿼터니언을 생성합니다.

		q_last_curr = q_identity.slerp(T(s), q_last_curr);
		// slerp를 사용하여 보간된 쿼터니언을 생성합니다.

		Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};
		// 스케일 s를 곱하여 변환 벡터 t_last_curr을 생성합니다.

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;
		// 쿼터니언과 변환 벡터를 적용하여 현재 포인트 cp를 새로운 좌표 lp로 변환합니다.

		residual[0] = (lp - lpj).dot(ljm);
		// 변환된 포인트와 평면 벡터 간의 내적을 계산하여 잔차(residual)에 저장합니다.

		return true;
		// 연산이 성공적으로 완료되었음을 반환합니다.
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_j_,
									   const Eigen::Vector3d last_point_l_, const Eigen::Vector3d last_point_m_,
									   const double s_)
	{
		// 팩토리 함수: LidarPlaneFactor 객체를 생성하고 Ceres Solver에서 사용할 수 있도록 포인터를 반환합니다.
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneFactor, 1, 4, 3>(
			new LidarPlaneFactor(curr_point_, last_point_j_, last_point_l_, last_point_m_, s_)));
	}

	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m; // 현재 포인트와 이전 세 포인트를 저장할 멤버 변수입니다.
	Eigen::Vector3d ljm_norm; // 이전 포인트들의 평면 법선 벡터를 저장할 멤버 변수입니다.
	double s; // 스케일 파라미터 s를 저장할 멤버 변수입니다.
};

struct LidarPlaneNormFactor
{
	// 생성자: 현재 점, 평면 단위 법선 벡터, 음수 OA 내적 값으로 초기화
	LidarPlaneNormFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_,
						 double negative_OA_dot_norm_) : curr_point(curr_point_), plane_unit_norm(plane_unit_norm_),
														 negative_OA_dot_norm(negative_OA_dot_norm_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		// 쿼터니언으로 회전 변환을 수행하기 위해 현재 회전 q를 Eigen 쿼터니언으로 변환
		// 매번 코드를 볼 때, 궁금한 거는 왜 쿼터니언의 순서가 3 0 1 2
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		
		// 현재 위치 변환 t를 Eigen 벡터로 변환
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		
		// 현재 점 cp를 템플릿 타입으로 변환하여 초기화
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		
		// 변환된 점을 저장할 point_w 벡터 정의
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;  // 회전과 이동을 적용하여 변환된 점 계산

		// 평면 법선 벡터 norm을 템플릿 타입으로 변환하여 정의
		Eigen::Matrix<T, 3, 1> norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));
		
		// 잔차 계산: 변환된 점과 평면 법선 벡터의 내적 + 음수 OA 내적 값
		residual[0] = norm.dot(point_w) + T(negative_OA_dot_norm);
		return true;
	}

	// 새로운 LidarPlaneNormFactor의 AutoDiffCostFunction을 생성하는 정적 함수
	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d plane_unit_norm_,
									   const double negative_OA_dot_norm_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarPlaneNormFactor, 1, 4, 3>(
			new LidarPlaneNormFactor(curr_point_, plane_unit_norm_, negative_OA_dot_norm_)));
	}

	// 현재 점의 좌표
	Eigen::Vector3d curr_point;

	// 평면의 단위 법선 벡터
	Eigen::Vector3d plane_unit_norm;

	// 평면과의 음수 OA 내적 값
	double negative_OA_dot_norm;
};

struct LidarDistanceFactor
{
	// 생성자: 현재 점과 가장 가까운 점으로 초기화
	LidarDistanceFactor(Eigen::Vector3d curr_point_, Eigen::Vector3d closed_point_) 
						: curr_point(curr_point_), closed_point(closed_point_) {}

	template <typename T>
	bool operator()(const T *q, const T *t, T *residual) const
	{
		// 쿼터니언으로 회전 변환을 수행하기 위해 현재 회전 q를 Eigen 쿼터니언으로 변환
		Eigen::Quaternion<T> q_w_curr{q[3], q[0], q[1], q[2]};
		
		// 현재 위치 변환 t를 Eigen 벡터로 변환
		Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
		
		// 현재 점 cp를 템플릿 타입으로 변환하여 초기화
		Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
		
		// 변환된 점을 저장할 point_w 벡터 정의
		Eigen::Matrix<T, 3, 1> point_w;
		point_w = q_w_curr * cp + t_w_curr;  // 회전과 이동을 적용하여 변환된 점 계산

		// 잔차 계산: 변환된 점과 가장 가까운 점 사이의 x, y, z 좌표 차이
		residual[0] = point_w.x() - T(closed_point.x());
		residual[1] = point_w.y() - T(closed_point.y());
		residual[2] = point_w.z() - T(closed_point.z());
		return true;
	}

	// 새로운 LidarDistanceFactor의 AutoDiffCostFunction을 생성하는 정적 함수
	static ceres::CostFunction *Create(const Eigen::Vector3d curr_point_, const Eigen::Vector3d closed_point_)
	{
		return (new ceres::AutoDiffCostFunction<
				LidarDistanceFactor, 3, 4, 3>(
			new LidarDistanceFactor(curr_point_, closed_point_)));
	}

	// 현재 점의 좌표
	Eigen::Vector3d curr_point;

	// 가장 가까운 점의 좌표
	Eigen::Vector3d closed_point;
};

