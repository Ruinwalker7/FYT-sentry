#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/String.h"
#include "rm_interfaces/Robots.h"
#include "rm_interfaces/Robot.h"
#include <ros/package.h>

namespace omni{
    struct robot{
        int camera_id;
        cv::Point2f tl;
        cv::Point2f br;
        int id;
    };

    struct object{
        int x;
        int y;
        int id;
    };

    struct MeasureGroup
    {
        double img1_time;
        double img2_time;
        double img3_time;
        double img4_time;

        std::vector<robot> cam1_robots;
        std::vector<robot> cam2_robots;
        std::vector<robot> cam3_robots;
        std::vector<robot> cam4_robots;
    };
    MeasureGroup group;


    
    struct Location{
        float yaw;
        int id;
        float distance;
    };
    static const char *labels[] = {"robot","red","blue","grey"};
    bool debug_;
    cv::VideoCapture capture1,capture2;
    float x1,x2,y1,y2;
    int id;
    std::vector<std::string> name ={"red","blue"};

    // pub_image.publish(cv_bridge::CvImage(hd, "rgb8", camera_img).toImageMsg());


    void Image1Callback(const sensor_msgs::ImageConstPtr &image1){
        cv::Mat src_image1 = cv_bridge::toCvShare(image1, "rgb8")->image;
        ros::Time time=ros::Time::now();

        if(debug_){
        cv::imshow("cam1",src_image1);
        cv::waitKey(1);
        }

    }
    
    void Location1Callback(const rm_interfaces::RobotsConstPtr &msg){
        std::vector<robot> cam1_robot;
        for(int i=0;i<msg->All_robots.size();i++){
            robot r;
            r.tl.y=msg->All_robots[i].tl.y;
            r.br.x=msg->All_robots[i].br.x;
            r.tl.x=msg->All_robots[i].tl.x;
            r.br.y=msg->All_robots[i].br.y;
            r.id=msg->All_robots[i].id;
            r.camera_id=1;
            cam1_robot.push_back(r);

        }
        group.img1_time=msg->header.stamp.toSec();
        group.cam1_robots=cam1_robot;
    }

    //统一每个相机检测到的机器
    bool sync_packages(MeasureGroup meas, std::vector<std::vector<robot>> &detected_objects)
    {
        std::vector<std::vector<robot>> detected_objects_temp;
        detected_objects_temp.push_back(meas.cam1_robots);
        detected_objects_temp.push_back(meas.cam2_robots);
        detected_objects_temp.push_back(meas.cam3_robots);
        detected_objects_temp.push_back(meas.cam4_robots);

        std::vector<double> src_times;
        src_times.push_back(meas.img1_time);
        src_times.push_back(meas.img2_time);
        src_times.push_back(meas.img3_time);
        src_times.push_back(meas.img4_time);

        bool syncFlag = false;

        for (int i = 0; i < src_times.size(); i++)
        {
            if (src_times[i] != 0 && (ros::Time::now().toSec() - src_times[i]) < 0.5)
            {
                detected_objects.push_back(detected_objects_temp[i]);
                syncFlag = true;
            }
            else
            {
                std::vector<robot> emptyObject;
                detected_objects.push_back(emptyObject);
                // ROS_WARN("Camera [%d] get image failed", i + 1);
            }
        }
        // if (!syncFlag)
        // {
        //     ROS_WARN("ALL Camera Failed, please check camera connection");
        // }
        return syncFlag;
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
    ros::Subscriber robots;
    bool showImg = true;
    ros_nh.param<std::string>("/rm_omni/yolo_model_path",model_path,"/YoloModel/0403");
    ros_nh.param("/debug",debug_,true);

    robots = ros_nh.subscribe<rm_interfaces::Robots>("/robots1",1,&omni::Location1Callback);
    // image_transport::ImageTransport it(ros_nh);
    // img_sub_ = ros_nh.subscribe<sensor_msgs::Image>("raw_img2", 1, &omni::Image1Callback);
    cv::Mat around_png = cv::imread(ros::package::getPath("rm_omni") + "/maps/around.png");
    while(ros::ok()){
        ros::spinOnce();
        std::vector<object> robot_result;
        std::vector<std::vector<robot>> detected_objects;
        //解算
        if(sync_packages(group,detected_objects)){
            for(int j=0;j<detected_objects.size();j++){
                std::vector<robot> obj = detected_objects[j];
                
                for(int i=0;i<obj.size();i++){
                    object temp;
                    int middle = (obj[i].br.x+obj[i].tl.x)/2;
                    int x = middle-176;
                    int y = 200;
                    if(j==1){
                        y = x;
                        x = 200;
                    }else if(j==2){
                        y=-y;
                    }
                    else if(j==3){
                        y=x;
                        x=-200;
                    }
                    temp.id=obj[j].id;
                    temp.x=x;
                    temp.y=y;
                    ROS_INFO("%d",x);
                    robot_result.push_back(temp);
                }
            }            
        }
        //可视化
        if(showImg){
            cv::Mat around = around_png.clone();
            // std::cout<<robot_result.size();
            for(int i=0;i<robot_result.size();i++){
                int x_around = -robot_result[i].x *1.5 + 300;
                int y_around = -robot_result[i].y   + 400;
                cv::putText(around, std::to_string(i+1) , cv::Point(x_around, y_around), 0, 1.5, cv::Scalar(0, 0, 255), 3, 8);
            }

            cv::imshow("around", around);
            cv::waitKey(1);
        }

    }
}