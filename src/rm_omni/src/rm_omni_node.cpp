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

    bool debug_;
    int id;
    std::vector<std::string> name ={"red","blue"};

    
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

    void Location2Callback(const rm_interfaces::RobotsConstPtr &msg){
        std::vector<robot> cam2_robot;
        for(int i=0;i<msg->All_robots.size();i++){
            robot r;
            r.tl.y=msg->All_robots[i].tl.y;
            r.br.x=msg->All_robots[i].br.x;
            r.tl.x=msg->All_robots[i].tl.x;
            r.br.y=msg->All_robots[i].br.y;
            r.id=msg->All_robots[i].id;
            r.camera_id=2;
            cam2_robot.push_back(r);
        }
        group.img2_time=msg->header.stamp.toSec();
        group.cam2_robots=cam2_robot;
    }

        void Location3Callback(const rm_interfaces::RobotsConstPtr &msg){
        std::vector<robot> cam3_robot;
        for(int i=0;i<msg->All_robots.size();i++){
            robot r;
            r.tl.y=msg->All_robots[i].tl.y;
            r.br.x=msg->All_robots[i].br.x;
            r.tl.x=msg->All_robots[i].tl.x;
            r.br.y=msg->All_robots[i].br.y;
            r.id=msg->All_robots[i].id;
            r.camera_id=1;
            cam3_robot.push_back(r);

        }
        group.img3_time=msg->header.stamp.toSec();
        group.cam3_robots=cam3_robot;
    }
        void Location4Callback(const rm_interfaces::RobotsConstPtr &msg){
        std::vector<robot> cam4_robot;
        for(int i=0;i<msg->All_robots.size();i++){
            robot r;
            r.tl.y=msg->All_robots[i].tl.y;
            r.br.x=msg->All_robots[i].br.x;
            r.tl.x=msg->All_robots[i].tl.x;
            r.br.y=msg->All_robots[i].br.y;
            r.id=msg->All_robots[i].id;
            r.camera_id=1;
            cam4_robot.push_back(r);

        }
        group.img4_time=msg->header.stamp.toSec();
        group.cam4_robots=cam4_robot;
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

        // ROS_INFO("%f,%d",src_times[1],detected_objects_temp[1].size());

        for (int i = 0; i < src_times.size(); i++)
        {
            if (src_times[i] != 0 && (ros::Time::now().toSec() - src_times[i]) < 1)
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
        return syncFlag;
    }
}



int main(int argc, char** argv){
    using namespace omni;
    ros::init(argc,argv,"rm_omni");
    ros::NodeHandle ros_nh;
    // 读参数
    std::string model_path;

    ros::Subscriber robots1_sub = ros_nh.subscribe<rm_interfaces::Robots>("/robots1",1,&omni::Location1Callback);
    ros::Subscriber robots2_sub = ros_nh.subscribe<rm_interfaces::Robots>("/robots2",1,&omni::Location2Callback);
    ros::Subscriber robots3_sub = ros_nh.subscribe<rm_interfaces::Robots>("/robots3",1,&omni::Location3Callback);
    ros::Subscriber robots4_sub = ros_nh.subscribe<rm_interfaces::Robots>("/robots4",1,&omni::Location4Callback);

    bool showImg = false;
    ros_nh.getParam("/rm_omni/showImg",showImg);
    ros_nh.param("/debug",debug_,true);

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
                    // ROS_INFO("%d",x);
                    robot_result.push_back(temp);
                }
            }            
        }
        //可视化
        if(showImg){
            cv::Mat around = around_png.clone();
            // std::cout<<robot_result.size();
            for(int i=0;i<robot_result.size();i++){
                int x_around = -robot_result[i].x  + 300;
                int y_around = -robot_result[i].y  + 400;
                cv::putText(around, std::to_string(i+1) , cv::Point(x_around, y_around), 0, 1.5, cv::Scalar(0, 0, 255), 3, 8);
            }

            cv::imshow("around", around);
            cv::waitKey(1);
        }

    }
}