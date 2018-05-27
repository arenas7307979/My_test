#include <iostream>
#include <cmath>
using namespace std;
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/so3.h"
#include "sophus/se3.h"
#include <map>
using namespace std;
int main(int argc, char**argv) {
#if 1
	// 沿Z轴转90度的旋转矩阵
	Eigen::Matrix3d R =
			Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

	Sophus::SO3 SO3_R(R);               // Sophus::SO(3)可以直接从旋转矩阵构造
	Sophus::SO3 SO3_v(0, 0, M_PI / 2);  // 亦可从旋转向量构造
	Eigen::Quaterniond q(R);            // 或者四元数
	Sophus::SO3 SO3_q(q);

	cout << "SO(3) from matrix =" << SO3_R << endl;
	cout << "SO(3) from vector =" << SO3_v << endl;
	cout << "SO(3) quaternion =" << SO3_q << endl;

	// Using mapping to get so3
	Eigen::Vector3d so3 = SO3_R.log();
	cout << "so3 = " << so3.transpose() << endl;
	//hat is convert vector to Anti-symmetry matrix
	cout << "so3 hat=" << Sophus::SO3::hat(so3) << endl;
	cout << "so3 hat vee=" << Sophus::SO3::vee(Sophus::SO3::hat(so3)) << endl;

	//Noise Update
	Eigen::Vector3d Update_so3(1e-4, 0, 0);
	Sophus::SO3 SO3_updated = Sophus::SO3::exp(Update_so3) * SO3_R;
	cout << "SO3_update=" << SO3_updated << std::endl;


	Eigen::Vector3d t(1,0,0);
    Sophus::SE3 SE3_Rt(R,t);
    Sophus::SE3 SE3_qt(q,t);
    cout << "SE3_Rt=" << SE3_Rt << std::endl;
    cout << "SE3_qt=" << SE3_qt << std::endl;



    typedef Eigen::Matrix<double, 6,1> vector6d;
    vector6d se3 = SE3_Rt.log();
    cout << "SE3 = " << se3.transpose() << std::endl;
    cout << "SE3 hat =" <<endl << Sophus::SE3::hat(se3) << std::endl;
    cout << "SE3 hat 2 =" <<endl <<Sophus::SE3::vee( Sophus::SE3::hat(se3) )<< std::endl;
    cout << "================add noise=================="  << std::endl;
    vector6d update_se3;
    update_se3.setZero();
    update_se3(0,0) = 1e-4d;
    Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3)*SE3_Rt;
    std::cout << "SE3 update = " << SE3_updated <<std::endl;

    cv::Mat FF;

    #endif

	return 0;
}
