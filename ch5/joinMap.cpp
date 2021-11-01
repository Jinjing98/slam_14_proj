#include <iostream>
#include <boost/format.hpp>
#include <fstream>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace std;
 
int main(int argc,char** argv){
    
    cout<<" This is joing pcd map!"<<endl;
 
    vector<cv::Mat> color_imgs, depth_imgs;
    vector<Eigen::Isometry3d> poses;
    
    fstream poses_txt("../pose.txt");
    if(!poses_txt){
        cout<<"the pose txt file path is not correct!"<<endl;
        return 1;
    }
    
//     Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
    for(int i= 0; i<5;i++){
        boost::format imgpath_fmt("../%s/%d.%s");
        color_imgs.push_back( cv::imread((imgpath_fmt%"color"%(i+1)%"png").str(),-1 ));
        depth_imgs.push_back( cv::imread((imgpath_fmt%"depth"%(i+1)%"pgm").str(),-1 ));  //  without -1, opencv will preprocess it to GBR CV_8UC3
        
        double data[7] = {0};
        for(auto& d:data){
            poses_txt>>d;    }
        Eigen::Quaterniond q(data[6],data[3],data[4],data[5]);
        Eigen::Isometry3d current_pose(q);
        current_pose.pretranslate(Eigen::Vector3d(data[0],data[1],data[2]));
        poses.push_back(current_pose);
       
    }
    
    
    
    
    
    // the intrisics of Cam
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double scale = 1000.0;//  mm to meter
    
    
    cv::imshow("",color_imgs[0]);
    cv::waitKey(0);
    cv::imshow("",depth_imgs[0]);
    cv::waitKey(0);
    std::cout<<"color: "<<color_imgs[0].type()<<" channels: "<<color_imgs[0].channels()<<endl;//CV_8UC3
    std::cout<<"depth: "<<depth_imgs[0].type()<<" channels: "<<depth_imgs[0].channels()<<endl;//CV_16UC1
    
    
//     Eigen::Quaterniond q(1,2,3,4);  // the init of q is w x y z, when print out q.coeffs(), the order is x y z w
//     cout<<q.coeffs()<<endl;

//construct point cloud
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PCD;
    
    PCD::Ptr pointcloud(new PCD);
    
    



    for(int i = 0; i<5;i++){
        cv::Mat color_img = color_imgs[i];
        cv::Mat depth_img = depth_imgs[i];
        Eigen::Isometry3d Twc = poses[i];
        std::cout<<"pose: "<<Twc.rotation()<<" \n"<<Twc.translation().transpose()<<endl;
        
        for(int y = 0; y<depth_img.rows;y++){
            uint8_t* color_rowPtr = color_img.ptr<uint8_t>(y);
            for(int x = 0; x<depth_img.cols;x++){
                 unsigned short d = depth_img.ptr<unsigned short>(y)[x];//depth_img.at<double>(y,x);
                 if(d==0) continue;
                 Eigen::Vector3d Pc((x-cx)/fx,(y-cy)/fy,1);
                 Pc = Pc*double(d)/scale;
                 Eigen::Vector3d Pw = poses[i]*Pc;
                 
                 PointT P_pcd;
                 P_pcd.x = Pw[0];
                 P_pcd.y = Pw[1];
                 P_pcd.z = Pw[2];
                 
                 P_pcd.b = color_rowPtr[x*3+0];
                 P_pcd.g = color_rowPtr[x*3+1];
                 P_pcd.r = color_rowPtr[x*3+2];
                 
                 pointcloud->points.push_back(P_pcd);
 
                
                
            }
                
                
                
        }

 
        
    }
    
    pointcloud->is_dense = false;
    cout<<" the PCD has num of points: "<<pointcloud->size()<<endl;
    pcl::io::savePCDFileBinary("../map.pcd",*pointcloud);
    
    
    
    Eigen::Quaterniond q2(0.966,-0.027,-0.25,-0.04);
    std::cout<<q2.toRotationMatrix()<<endl;
    
 
    
    return 0;
}
