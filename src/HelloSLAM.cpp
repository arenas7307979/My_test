#include <iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <opencv2/opencv.hpp>
using namespace cv;
cv::Mat im1 = imread("1.png", CV_BGR2GRAY);
cv::Mat im2 = imread("2.png", CV_BGR2GRAY);
cv::Mat im3 = imread("office-03.png", CV_BGR2GRAY);
void KnnMatches(std::vector<cv::KeyPoint> &kps_set1,
		std::vector<cv::KeyPoint> &kps_set2, cv::Mat &desc1, cv::Mat &desc2,
		std::vector<std::vector<DMatch>> &matches,
		std::vector<cv::KeyPoint> &Newkps1, std::vector<cv::KeyPoint> &Newkps2,
		cv::Mat &Newdesc1, cv::Mat &Newdesc2) {
	float nn_match_ratio = 0.7f;
	Newkps1.clear();
	Newkps2.clear();
	cv::Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	matcher->knnMatch(desc1, desc2, matches, 2);
	for (unsigned i = 0; i < matches.size(); i++) {
		if (matches[i][0].distance < nn_match_ratio * matches[i][1].distance) {
			Newkps1.push_back(kps_set1[matches[i][0].queryIdx]);
			Newkps2.push_back(kps_set2[matches[i][0].trainIdx]);
			Newdesc1.push_back(desc1.row(matches[i][0].queryIdx));
			Newdesc2.push_back(desc2.row(matches[i][0].trainIdx));
		}
	}
	cv::Mat tmp11;
	cv::drawMatches(im1,kps_set1, im2,kps_set2,matches,tmp11);
	imshow("1",tmp11);
	cv::waitKey(0);
}
void FindMultiMatches(std::map<int, std::vector<cv::KeyPoint>> &KpsSet,
		std::map<int, cv::Mat> &DescSet, int frame_num,
		std::map<int, std::vector<cv::KeyPoint>> MatchKpsSet,
		std::map<int, cv::Mat> &MatchDescSet) {
	for (int i = 0; i < 1; i++) {
		std::vector<std::vector<DMatch>> matches;
		std::vector<cv::KeyPoint> kps_new1;
		std::vector<cv::KeyPoint> kps_new2;
		cv::Mat Newdesc1, Newdesc2;
		KnnMatches(KpsSet[i], KpsSet[i + 1], DescSet[i], DescSet[i + 1],
				matches, kps_new1, kps_new2, Newdesc1, Newdesc2);
	}

}

int main(int argc, char ** argv) {
//=======================Input image ============================================
//	cv::Mat im1 = imread("office-00.png", CV_BGR2GRAY);
//	cv::Mat im2 = imread("office-02.png", CV_BGR2GRAY);
//	cv::Mat im3 = imread("office-03.png", CV_BGR2GRAY);
	std::vector<cv::Mat> imgset;
	imgset.push_back(im1);
	imgset.push_back(im2);
	imgset.push_back(im3);
//=======================Extractor Descr and Feature ============================
	cv::Ptr<ORB> orbfeature = cv::ORB::create(200);
	std::map<int, std::vector<cv::KeyPoint>> keypointset;
	std::map<int, cv::Mat> Descriptorset;

//Descriptorset.resize(imgset.size());
	for (int i = 0; i < imgset.size(); i++) {
		std::vector<cv::KeyPoint> tmpk;
		cv::Mat tmpDes;
		orbfeature->detectAndCompute(imgset[i], cv::noArray(), tmpk, tmpDes);
		Descriptorset[i] = tmpDes;
		keypointset[i] = tmpk;
	}

//=======matching std image, out map <int, vector<keypoint>>
	std::map<int, std::vector<cv::KeyPoint>> NewKeySet;
	std::map<int, cv::Mat> NewDescriptorset;
	FindMultiMatches(keypointset,Descriptorset,3,NewKeySet,NewDescriptorset);
	return 0;
}
