#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/String.h"




namespace omni{
    static const char *labels[] = {"robot","red","blue","grey"};
    bool debug_;
    cv::VideoCapture capture1,capture2;

    // pub_image.publish(cv_bridge::CvImage(hd, "rgb8", camera_img).toImageMsg());


    void Image1Callback(const sensor_msgs::ImageConstPtr &image1){
        cv::Mat src_image1 = cv_bridge::toCvShare(image1, "rgb8")->image;
        ros::Time time=ros::Time::now();

        if(debug_){
        cv::imshow("cam1",src_image1);
        cv::waitKey(1);
        }

    }

    void Image2Callback(){
        capture1.set(cv::CAP_PROP_FRAME_HEIGHT,320);
        capture1.set(cv::CAP_PROP_FRAME_WIDTH,240);
        capture1.set(cv::CAP_PROP_FPS,30);
        capture1.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('M','J','P','G'));
        capture1.open(-1);
        if(!capture1.isOpened()){
            ROS_INFO("Camera2 Error!");
            return ;
        }

    }
    
}



int main(int argc, char** argv){
    using namespace omni;
    ros::init(argc,argv,"rm_omni");
    std::string img1_topic_name;
    ros::Subscriber img_sub_;
    ros::NodeHandle ros_nh;
    // 读参数
    std::string model_path;
    
    ros_nh.param<std::string>("/rm_omni/yolo_model_path",model_path,"/YoloModel/0403");
    ros_nh.param("/debug",debug_,true);


    image_transport::ImageTransport it(ros_nh);
    img_sub_ = ros_nh.subscribe<sensor_msgs::Image>("raw_img2", 1, &omni::Image1Callback);


    // Image2Callback();
    // Image3Callback();


    while(ros::ok()){
        ros::spinOnce();
        // capture>>frameImg;
        // if(!frameImg.empty()&&debug_){
        //     cv::imshow("frameImg",frameImg);
        //     cv::waitKey(30);
        // }
    }

}