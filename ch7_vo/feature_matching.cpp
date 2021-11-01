#include <iostream>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
int main(int argc, char** argv){

	cout<<"This is testing feature extraction and matching!"<<endl;
    
    cv::Mat img1,img2;
    img1 = cv::imread("../data/up.jpg",-1);
    img2 = cv::imread("../data/down.jpg",-1);
//     cv::imshow("",img1);
//     cv::waitKey(0);
//     cv::imshow("",img2);
//     cv::waitKey(0);
    std::cout<<"img format: "<<img1.channels()<<" "<<img1.type()<<endl;//  16   CV_8UC3
    
    
    // ORB feature detector
    vector<cv::KeyPoint> kps_1,kps_2;
    cv::Mat des_1,des_2;
    cv::Ptr<cv::ORB> orb = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE,31,20);
    cv::Ptr<cv::ORB> orb2 = cv::ORB::create(100, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE,31,20);
    orb->detect(img1,kps_1);
    orb2->detect(img2,kps_2);
    
    orb->compute(img1,kps_1,des_1);
    orb2->compute(img2,kps_2,des_2);
    
    
    
    std::cout<<"img1: "<<kps_1.size()<<" "<<des_1.rows<<" "<<des_1.cols<<endl;
    std::cout<<"img2: "<<kps_2.size()<<" "<<des_2.rows<<" "<<des_2.cols<<endl;
    
    
    double max_dis = 0;
    double min_dis = 100000;
    
    
    // we use BF matcher
    cv::BFMatcher matcher;
    vector<cv::DMatch> matches;
//     matcher.match(des_1,des_2,matches);
    
    // always regard the descriptor in the first posistion as reference, i.e, matches.size() == des_2.rows()
    matcher.match(des_2,des_1,matches);
    
    for(auto match:matches){
        if(match.distance<min_dis)
            min_dis = match.distance;// the query idx and train idx gets reserved
    }
    std::cout<<"the min dis: "<<min_dis<<endl;
    std::cout<<"the num of matches: "<<matches.size()<<endl;
    
    vector<DMatch> good_matches;
    
    for(auto match:matches){
        if(match.distance<=  max(4*min_dis,30.0))
            good_matches.push_back(match);
        
    }
    std::cout<<"the num of good_matches: "<<good_matches.size()<<endl;
    
    cv::Mat new1,new2,img_match;
    cv::drawKeypoints(img1,kps_1,new1);
    cv::drawKeypoints(img2,kps_2,new2);
    cv::imshow("",new1);
    cv::waitKey(0);
    cv::imshow("",new2);
    cv::waitKey(0);
    
    // this order is alighed with the order in "matcher.match(des_2,des_1,matches);"
    cv::drawMatches(img2,kps_2,img1,kps_1,matches,img_match);
    cv::imshow("",img_match);
    cv::waitKey(0);
    
    cv::drawMatches(img2,kps_2,img1,kps_1,good_matches,img_match);
    cv::imshow("",img_match);
    cv::waitKey(0);    
    
    
    
    
    
    
    
    
    
    
    
    
    
	

	return 0;

}
