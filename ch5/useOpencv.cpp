#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
int main(int argc, char** argv){

    std::cout<<"This is the test for Opencv!"<<endl;
 


    cv::Mat img;
    img = cv::imread(argv[1]);
    if(img.data == nullptr){  // img.data is ptr type
        std::cout<<"there is no img!"<<endl;
        return 0;
    }
    
    
    if(img.type()!= CV_8UC1 && img.type()!= CV_8UC3){
        cout<<"This is not a gray/RGB img!"<<endl;
        return 0;
    }
    
    cv::imshow(" ",img);
    cv::waitKey(0);
    
    // about different types: https://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv
    std::cout<<"cols: "<<img.cols<<" rows: "<<img.rows<<" channels: "<<img.channels()<<" data type:"<<img.type()<<endl;
    
    //access values of img  ; datatype  unsigned char 8 bit
    // three different ways: https://stackoverflow.com/questions/7899108/opencv-get-pixel-channel-value-from-mat-image
    
    //reverse the image
    cv::Mat new_img = img.clone();
    int height = img.rows;
    int width = img.cols;
    
    
    
    // flip the images by access the pixel values one by one
    int num4channels = img.channels();
    cv::Vec3b pixel_vals;
    for(int y = 0; y<img.rows;y++){
        uint8_t* rowPtr = img.ptr<uint8_t>(y);//img.row(y).data;  can be used as the RHS too!  CF: new_img.at<cv::Vec3b>(height-y,width-x)
        for(int x= 0; x<img.cols;x++){
            pixel_vals.val[0] = rowPtr[x*num4channels+0];
            pixel_vals.val[1] = rowPtr[x*num4channels+1];
            pixel_vals.val[2] = rowPtr[x*num4channels+2];
//             unsigned char pixel = img.at()
            new_img.at<cv::Vec3b>(height-y,width-x) = pixel_vals;//.cast()<>;
            
        }
    }
    cv::imshow(" ",new_img);
    cv::waitKey(0);
    
    
    
    
    cv::Mat img2 = img;
    img2(cv::Rect(0,0,100,200)).setTo(0);
    cv::imshow(" ",img);
    cv::waitKey(0);
    
    
    cv::Mat img3 = img.clone();
    img3(cv::Rect(100,400,50,50)).setTo(1);
    cv::imshow(" ",img);
    cv::waitKey(0);
    cv::imshow(" ",img3);
    cv::waitKey(0);
    
    
    return 0;





 
}
