#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>




using namespace std;
using namespace cv;

inline cv::Scalar get_color(float depth) {
  float up_th = 50, low_th = 10, th_range = up_th - low_th;
  if (depth > up_th) depth = up_th;
  if (depth < low_th) depth = low_th;
  return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}

void find_feature_matches(
  const Mat &img1, const Mat &img2,
  std::vector<cv::KeyPoint> &kps_1,
  std::vector<cv::KeyPoint> &kps_2,
  std::vector<cv::DMatch> &good_matches){
      
    
    // ORB feature detector
//     vector<cv::KeyPoint> kps_1,kps_2;
    cv::Mat des_1,des_2;
//     cv::Ptr<cv::ORB> orb = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE,31,20);
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
//     cv::Ptr<cv::ORB> orb2 = cv::ORB::create(100, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE,31,20);
    detector->detect(img1,kps_1);
    detector->detect(img2,kps_2);
    
    descriptor->compute(img1,kps_1,des_1);
    descriptor->compute(img2,kps_2,des_2);
    
    
    
    std::cout<<"img1: "<<kps_1.size()<<" "<<des_1.rows<<" "<<des_1.cols<<endl;
    std::cout<<"img2: "<<kps_2.size()<<" "<<des_2.rows<<" "<<des_2.cols<<endl;
    
    
    double max_dis = 0;
    double min_dis = 100000;
    
    
    // we use BF matcher
//     cv::BFMatcher matcher;
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    vector<cv::DMatch> matches;
//     matcher.match(des_1,des_2,matches);
    
    // always regard the descriptor in the first posistion as reference, i.e, matches.size() == des_2.rows()
    matcher->match(des_1,des_2,matches);
    
    for(auto match:matches){
        if(match.distance<min_dis)
            min_dis = match.distance;// the query idx and train idx gets reserved
        if(match.distance>max_dis)
            max_dis = match.distance;
    }
    std::cout<<"the min max dis: "<<min_dis<<" "<<max_dis<<endl;
    
//     std::cout<<"the num of matches: "<<matches.size()<<endl;
    
//     vector<DMatch> good_matches;
    
    for(auto match:matches){
        if(match.distance<=  max(2*min_dis,30.0))
            good_matches.push_back(match);
        
    }
    
    
    
}


void pose_estimation_2d2d(
  std::vector<cv::KeyPoint> kps_1,
  std::vector<cv::KeyPoint> kps_2,
  std::vector<cv::DMatch> good_matches,
  Mat &R, Mat &t){

//esitimated E F H  wrt K, matches kps
    
    Mat K = (cv::Mat_<double>(3,3)<<520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point2d> pts1,pts2;
    for(auto match:good_matches){
        pts1.push_back(kps_1[match.queryIdx].pt);
        pts2.push_back(kps_2[match.trainIdx].pt);
    }
    
    
    //F
    Mat F,E,H;
    F = cv::findFundamentalMat(pts1,pts2,FM_8POINT);
    E = cv::findEssentialMat(pts1,pts2,521.0,Point2d(325.1,249.7));
    H = cv::findHomography(pts1,pts2,RANSAC,3);//  the 3 is a RANSACreprojection trd value
    
    
    std::cout<<"the estimated F E H are: \n"<<F<<"\n"<<E<<"\n"<<H<<endl;
    
    
    
    // recover R,t
    
    cv::recoverPose(E,pts1,pts2,R,t,521.0,Point2d(325.1,249.7));
    std::cout<<" the recovered R and t are :\n"<<R<<"\n"<<t<<endl;
    
 
    
}






cv::Point2f pixel2cam(const cv::Point2d &p, const Mat &K){
 
    Eigen::Matrix3d K_eigen;
    cv2eigen(K,K_eigen);
    Eigen::Vector3d p_eigen(p.x,p.y,1);
    Eigen::Vector3d p_norm_eigen = K_eigen.inverse()*p_eigen;
    
    
//     cout<<"debug: "<<K_eigen<<" "<<K_eigen.inverse()<<" "<<p_eigen<<endl;
//     cout<<" new: "<<p_norm_eigen.transpose()<<endl;
    
    return Point2f(p_norm_eigen[0],p_norm_eigen[1]);
 
    
    
}





void triangulation(
  const vector<KeyPoint> &kps_1,
  const vector<KeyPoint> &kps_2,
  const std::vector<DMatch> &good_matches,
  const Mat &R, const Mat &t,
  vector<Point3d> &points){
      
    vector<Point2d> pts1,pts2;
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for(auto match:good_matches){
        pts1.push_back(pixel2cam( kps_1[match.queryIdx].pt,K));
        pts2.push_back(pixel2cam( kps_2[match.trainIdx].pt,K));
    }
    cv::Mat pos1 = (cv::Mat_<double>(3,4)<<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0
    );
    cv::Mat pos2 = (cv::Mat_<double>(3,4)<<
    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)       
    );
    Mat pts_4d;
    cv::triangulatePoints(pos1,pos2,pts1,pts2,pts_4d);//  it requirs the pts1 pts2 in float format
    std::cout<<"pts_4d data type : "<<pts_4d.channels()<<" "<<pts_4d.type()<<" "<<pts_4d.rows<<" "<<pts_4d.cols<<endl;//   CV_64FC1
    for(int i = 0; i < pts_4d.cols;i++){
        Mat col = pts_4d.col(i);
        col = col/col.at<float>(3,0);
        std::cout<<"debug: "<<col.at<float>(3,0)<<" "<<col.at<double>(3,0)<<endl;
        Point3d p(col.at<float>(0,0),col.at<float>(1,0),col.at<float>(2,0));
//         std::cout<<"debug: "<<p.x<<" "<<p.y<<" "<<p.z<<endl;
        points.push_back(p);
    }
    
    
}



int main(int argc, char** argv){
    
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
//     Point2d p(1,2);
//     pixel2cam(p,K);
    
    cv::Mat img1,img2;
    img1 = cv::imread("../data/1.png",-1);
    img2 = cv::imread("../data/2.png",-1);
    vector<KeyPoint> kps_1,kps_2;
    vector<DMatch> good_matches;
    find_feature_matches(img1,img2,kps_1,kps_2,good_matches);
    cout<<"the num of good matches :"<<good_matches.size()<<endl;
    
    cv::Mat R,t;
    pose_estimation_2d2d(kps_1,kps_2,good_matches,R,t);
    
    

    
    
    
    // test if the recoverd R t are correct    E = t^R
    Mat t_hat = (cv::Mat_<double>(3,3)<<
        0, -t.at<double>(2, 0), t.at<double>(1, 0),
        t.at<double>(2, 0), 0, -t.at<double>(0, 0),
        -t.at<double>(1, 0), t.at<double>(0, 0), 0
    );
    std::cout<<"the computed E is :"<<t_hat*R<<endl;//  it turned out to be up to a scale! correct!
    
    
    // test if for each pair of matched point, the epipolar restriction exist
    
    for(auto match:good_matches){
        cv::Mat x1 = (cv::Mat_<double>(3,1)<<pixel2cam(kps_1[match.queryIdx].pt,K).x,pixel2cam(kps_1[match.trainIdx].pt,K).y,1);    
        cv::Mat x2 = (cv::Mat_<double>(3,1)<<pixel2cam(kps_2[match.trainIdx].pt,K).x,pixel2cam(kps_2[match.queryIdx].pt,K).y,1);
        
        cout<<x2.t()*t_hat*R*x1<<endl;
    }


     
    cv::Mat img1_kp,img2_kp;
    cv::drawKeypoints(img1,kps_1,img1_kp);
    cv::drawKeypoints(img2,kps_2,img2_kp);
    hconcat(img1_kp,img2_kp,img2_kp);
    cv::imshow(" ",img2_kp);
    cv::waitKey(0); 
    
    
/*    
    
//triangulation points 
//given pos1 pos2  pts1 matched_pts2  we get the pts4d wrt frame1    
    vector<Point3d> pts3d_cam1,pts3d_cam2;
    triangulation(kps_1,kps_2,good_matches,R,t,pts3d_cam1);
    // now we have 3d pos of points in frame1 saved in pts3d
    // now we project the point in cam1 in the cam2, then get the pixel pos of these points in frame2;
    for(auto p:pts3d_cam1){
        cv::Mat p_mat = (cv::Mat_<double>(3,1)<<p.x,p.y,p.z);
//         std::cout<<"debug : "<<p.x<<" "<<p.y<<" "<<p.z<<endl;
//         cout<<"test2: "<<p_mat.type()<<endl;
        cv::Mat p2_mat = R*p_mat+t;
//         std::cout<<"test p2 mat: "<<p2_mat.type()<<" "<<p_mat.type()<<endl;
        Point3d p2(p2_mat.at<double>(0,0),p2_mat.at<double>(1,0),p2_mat.at<double>(2,0));
//         std::cout<<"debug : "<<p2<<endl;
        pts3d_cam2.push_back(p2);
    }
    

    cv::Mat proj2_wrt_trian = img2.clone();
    std::cout<<"test3: "<<pts3d_cam2.size()<<endl;
    for(auto p:pts3d_cam2){
        cv::Mat pt = (cv::Mat_<float>(3,1)<<p.x/p.z,p.y/p.z,1);
        cv::Mat pixel = K*pt;
//         std::cout<<"pixel"<<pixel<<endl;
        cv::circle(proj2_wrt_trian,Point2d(pixel.at<double>(0,0),pixel.at<double>(1,0)),5,get_color(10));
//         pixels_cam2.push_back(Point2d(pixel.at<double>(0,0),pixel.at<double>(1,0)));
//         std::cout<<"test3: "<<pixel.rows<<" "<<pixel.cols<<endl;
        
    }
    
 
    cv::imshow("",proj2_wrt_trian);
    cv::waitKey(0);
//  
    
 
    
 */
    
    
    
    
    
    
    
    return 0;
}
