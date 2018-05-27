#include <iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <opencv2/opencv.hpp>
using namespace cv;

int main(int argc, char**argv) {

	//initial p
	float p1 = 0, p2 = 0, p3 = 0, p4 = 0, p5 = 0, p6 = 0;

	cv::Mat transformMatrix = (cv::Mat_<float>(2, 3) << 1, 0, 0, 0, 1, 0);
	cv::Mat im1 = imread("office-00.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat Template = imread("part2.png", CV_LOAD_IMAGE_GRAYSCALE);

	//image diff as measurement
	cv::Mat im1_part = im1.rowRange(115, 315).colRange(115, 315);


float init_x = 115;
float init_y = 115;
	for (int t = 0; t <= 30; t++) {
#if 1
		cv::Mat error = Template - im1_part;

		//sobel image gradient Ix, Iy
		cv::Mat sobel_x_im1_part;
		cv::Mat sobel_y_im1_part;
		cv::Sobel(im1_part, sobel_x_im1_part, CV_32F, 1, 0, 3);
		cv::Sobel(im1_part, sobel_y_im1_part, CV_32F, 0, 1, 3);
		//

		cv::Mat Hessi_ACC = cv::Mat::zeros(6, 6, CV_32F); //初始值为0阵
		cv::Mat totalDesc = cv::Mat::zeros(6, 1, CV_32F); //初始值为0阵，其维数为（6*1）*1
		cv: Mat JacobiW;
		for (int i = 0; i < Template.rows; i++) {
			for (int j = 0; j < Template.cols; j++) {
				cv::Mat error = cv::Mat::zeros(1, 1, CV_32F);
				JacobiW =
						(cv::Mat_<float>(2, 6) << j, 0, i, 0, 1, 0, 0, j, 0, i, 0, 1);

				cv::Mat GradientXY =
						(cv::Mat_<float>(1, 2)
								<< sobel_x_im1_part.at<float>(j, i), sobel_y_im1_part.at<
								float>(j, i));

				cv::Mat GradientXY_jacobian = GradientXY * JacobiW;
				cv::Mat Hessi = GradientXY_jacobian.t() * GradientXY_jacobian;
				Hessi_ACC = Hessi + Hessi_ACC;

				error = (cv::Mat_<float>(1, 1)
						<< (float) Template.at<uchar>(j, i)
								- (float) im1_part.at<uchar>(j, i));

				totalDesc = (GradientXY_jacobian.t() * error) + totalDesc; //6*1

			}
		}

		//update dela p
		cv::Mat deltaP = Hessi_ACC.inv() * totalDesc;
		std::cout << deltaP << std::endl;
		p1 = p1 + deltaP.at<float>(0, 0);
		p2 = p2 + deltaP.at<float>(0, 1);
		p3 = p3 + deltaP.at<float>(0, 2);

		p4 = p4 + deltaP.at<float>(0, 3);
		p5 = p5 + deltaP.at<float>(0, 4);
		p6 = p6 + deltaP.at<float>(0, 5);
		//warp source image
#if 1
		cv::Mat warp_dst;
		//cv::Mat warp_dst =  Mat::zeros( im1.rows, im1.cols, im1.type() );//
		cv::Mat affine_update = (cv::Mat_<float>(2, 3) << 1 + p1, p3, p5, p2, 1
				+ p4, p6);
		cv::Mat Point2d = (cv::Mat_<float>(3, 1) << init_x , init_y, 1) ;

		cv::Mat Point2d_newWarp = affine_update * Point2d;
		std::cout << "Point2d_newWarp" << Point2d_newWarp <<std::endl;
		init_x = Point2d_newWarp.at<float>(0, 0);
		init_y = Point2d_newWarp.at<float>(0, 1);
		warpAffine(im1, warp_dst, affine_update, im1.size());
		cv::Mat im1_part = warp_dst.rowRange(init_x, init_x+200).colRange(init_y, init_y+200);

#endif

		std::cout << "p1=" << p1 << " p2=" << p2 << " p3=" << p3 << " p4=" << p4
				<< " p5=" << p5 << " p6=" << p6;
#endif

//	std::cout << error <<std::endl;
		cv::imshow("Template", Template);
		cv::imshow("im1_part", im1_part);
//	cv::imshow("Template",Template);
//	cv::imshow("error",error);
		//cv::imwrite("part2.png", im1_part);
		cv::waitKey(0);
	} // iterator 10 times
	return 0;
}
