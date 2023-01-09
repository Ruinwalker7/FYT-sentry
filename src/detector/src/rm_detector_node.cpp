// Copyright FYT 邹承甫 2022

#include "detector/rm_detector_node.hpp"

namespace detector
{

DetectorNode::DetectorNode()
: nh_("~")
{

  nh_.param("/debug",debug_,true);
  nh_.param("/enemy_color",enemy_color_,1);

  std::string model_path;
  std::string config_path;
  std::string class_path;
  nh_.getParam("/detector/model_path",model_path);
  nh_.getParam("/detector/config_path",config_path);
  nh_.getParam("/detector/class_path",class_path);
  std::vector<std::string> files{class_path,model_path,config_path};


  int input_width, input_height;
  nh_.param("/detector/input_width",input_width,416);
  nh_.param("/detector/input_height",input_height,416);
  cv::Size input_size{input_width,input_height};

  float conf_thresh, nms_thresh;
  nh_.param("/detector/conf_thresh", conf_thresh, (float)0.2);
  nh_.param("/detector/nms_thresh", nms_thresh, (float)0.2);

  int device;
  nh_.param("/device",device,(int)device_type::OTHERS);

  detector_ = std::make_unique<Detector>(
    files,
    input_size,
    device,
    conf_thresh,
    nms_thresh,
    debug_
  );
  //*********ONNX请打开这里
  // onnx_detector_ = std::make_unique<onnxDetector>(
  //   files[1],
  //   conf_thresh,
  //   nms_thresh,
  //   cv::Size{640,640}
  // );

  img_sub_ = nh_.subscribe<sensor_msgs::Image>(
    "/raw_img", 1, &DetectorNode::image_cb, this);
  track_info_sub_ = nh_.subscribe<rm_interfaces::TrackInfo>(
    "/track_info", 1, &DetectorNode::track_info_cb, this);
  targets_pub_ = nh_.advertise<rm_interfaces::AllTargets>(
   "/all_targets",1);
  
}

void DetectorNode::track_info_cb(const rm_interfaces::TrackInfoConstPtr &msg)
{
  track_info_ = *msg;
}

void DetectorNode::image_cb(const sensor_msgs::ImageConstPtr &msg)
{
  cv::Mat src = cv_bridge::toCvCopy(msg,"bgr8")->image;
  ros::Time stamp = msg->header.stamp;
  cv::namedWindow("4pts",cv::WINDOW_AUTOSIZE);
  if(src.empty())
  {
    return;
  }
  std::vector<cv::Rect> box;
  std::vector<int> id;
  int offset = 1; 
  // 选取roi进行识别，不浪费视野
  if(src.rows == 1280 && src.cols == 600 && track_info_.is_tracking)
  {
    if(track_info_.RPY.z < 0)
    {
      offset = 2;
      cv::Rect left_roi(480,0,800,600);
      src = src(left_roi);
    }
    else if(track_info_.RPY.z > 0)
    {
      offset = 0;
      cv::Rect left_roi(0,0,800,600);
      src = src(left_roi);
    }
    else
    {
      offset = 1;
      cv::Rect center_roi(240,0,800,600);
      src = src(center_roi);
    }
  }
  detector_->Detecting(src,box,id);
  // auto detections = onnx_detector_->Detect(src);

  rm_interfaces::AllTargets allTargets;
  //**************onnx的话请取消以下注释*****************
  // for(auto detection : detections)
  // {
  //   if((enemy_color_==0 && (detection.id<8 ||detection.id>15))
  //   ||(enemy_color_==1 && detection.id < 17))
  //   {
  //       continue;
  //   }
  //   // int number = (detection.id%9+2)%9;
  //   rm_interfaces::Armor armor;
  //   armor.id = detection.id;
  //   armor.tl.x = detection.points[0].x;
  //   armor.tl.y = detection.points[0].y;
  //   armor.bl.x = detection.points[1].x;
  //   armor.bl.y = detection.points[1].y;
  //   armor.br.x = detection.points[2].x;
  //   armor.br.y = detection.points[2].y;
  //   armor.tr.x = detection.points[3].x;
  //   armor.tr.y = detection.points[3].y;
  //   allTargets.armors.emplace_back(armor);
  // }
  //********************darknet的话请取消以下注释************************          
  for(int i = 0;i<id.size();++i)
  {
    if((enemy_color_==0 && (id[i]<8 ||id[i]>15))
    ||(enemy_color_==1 && id[i] < 17))
    {
        continue;
    }
    int number = (id[i]%9+2)%9;

    box[i].x=box[i].x-0.2*box[i].width < 0 ? 0 : box[i].x-0.2*box[i].width;
    box[i].y = box[i].y-0.2*box[i].height < 0 ? 0 : box[i].y-0.2*box[i].height;
    box[i].width= box[i].x+box[i].width*1.4 > src.cols ? src.cols-box[i].x : box[i].width*1.4;
    box[i].height= box[i].y+box[i].height*1.4 > src.rows ? src.rows-box[i].y : box[i].height*1.4;

    cv::Point2f origin{box[i].x,box[i].y};
    if(box[i].x<0 || box[i].y<0 
    ||box[i].x+box[i].width>src.cols 
    || box[i].y+box[i].height>src.rows
    ||box[i].width == 0
    ||box[i].height== 0)
    {
        ROS_WARN("roi out of range\n");
        return;
    }

    cv::Rect roi_rect(box[i].x, box[i].y, box[i].width, box[i].height);
    cv::Mat roi = src(box[i]);
    
    if(debug_)
    {
      cv::imshow("armor",roi);
      cv::waitKey(1);
    }
    std::vector<cv::Point2f> img_pts;
    if(detect4pts(roi,img_pts,origin))
    {
      if(img_pts.size()!= 4)
      {
        return;
      }
      rm_interfaces::Armor armor;
      armor.id = number;
      armor.tl.x = img_pts[0].x;
      armor.tl.y = img_pts[0].y;
      armor.bl.x = img_pts[1].x;
      armor.bl.y = img_pts[1].y;
      armor.br.x = img_pts[2].x;
      armor.br.y = img_pts[2].y;
      armor.tr.x = img_pts[3].x;
      armor.tr.y = img_pts[3].y;
      allTargets.armors.emplace_back(armor);
    }
  }
  //*****************************************************************
  allTargets.header = std_msgs::Header();
  allTargets.header.stamp = stamp;
  rm_interfaces::Point2d src_offset;
  src_offset.x -= offset*240; 
  allTargets.offset = src_offset;
  targets_pub_.publish(allTargets);
}



bool DetectorNode::detect4pts(cv::Mat &roi, std::vector<cv::Point2f> &img_pts,
  const cv::Point2f &origin_pt)
{

  cv::Mat bin;
  cv::Mat bgr[3];
  cv::split(roi, bgr);
  if(enemy_color_ == 0)
  {
    cv::threshold(bgr[2], bin, 145, 255, cv::THRESH_BINARY);
  }
  else
  {
    cv::threshold(bgr[0], bin, 145, 255, cv::THRESH_BINARY);
  }
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
  cv::morphologyEx(bin,bin,cv::MORPH_CLOSE,kernel,cv::Point(-1,-1),1);

  if(debug_)
  {
    cv::imshow("bin",bin);
    cv::waitKey(1);
  }

  std::vector<std::vector<cv::Point>> contours; /*颜色轮廓*/
  cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if(contours.empty()) return -1;
    /*找灯条*/
    std::vector<cv::RotatedRect> lights;
    for(auto contour : contours){
        cv::RotatedRect single_light=cv::minAreaRect(contour);

        /*面积小于阈值*/
        if(single_light.size.area() < 20)
            continue;
        /*长宽比例小于阈值*/
        float ratio = single_light.size.width > single_light.size.height 
            ? single_light.size.width/single_light.size.height 
            : single_light.size.height/single_light.size.width;
        if(ratio < 1.2)
            continue;

        /*垂直矩形的高度大于宽度，且面积大于最小值*/
        cv::Rect bounding_rect = single_light.boundingRect();/*得到一个垂直的矩形*/
        if (bounding_rect.height < bounding_rect.width)    
            continue;

        /*防止roi越界*/
        if(0<=bounding_rect.x && 0<=bounding_rect.width && bounding_rect.width+bounding_rect.x<=roi.cols &&
        0<=bounding_rect.y && 0<=bounding_rect.height && bounding_rect.y+bounding_rect.height <= roi.rows)
        {
            /*判断灯条的颜色*/
            int sum_r = 0,sum_b = 0,sum_g = 0;
            cv::Mat tmp = roi(bounding_rect);
            uchar* ptr_tmp = tmp.data;
            for(int i = 0;i<tmp.rows*tmp.cols;++i)
            {
                sum_b += *ptr_tmp;
                sum_g += *(++ptr_tmp);
                sum_r += *(++ptr_tmp);
                ++ptr_tmp;
            }
            if(sum_b - sum_r > 20||sum_r - sum_b > 20)
            {
                lights.emplace_back(single_light);
            }
        }
        else
        {
            continue;
        }
        
    }
    /*寻找装甲板的四个角点
     *顺序为tl,bl,tr,br*/
    if(lights.size() == 2)
    {
        std::sort(lights.begin(),lights.end(),
        [](const cv::RotatedRect & a,const cv::RotatedRect &b) {return a.center.x < b.center.x;});
        std::vector<cv::Point2f> tmp_pts;
        for(auto light : lights)
        {
            cv::Point2f pts[4];
            light.points(pts);
            std::sort(pts,pts+4,[](const cv::Point2f & a,const cv::Point2f &b) {return a.y < b.y;});
            auto top = (pts[0] + pts[1])/2;
            auto buttom = (pts[2] + pts[3])/2;
            if(debug_){
                cv::circle(roi,top,2,{0,255,0},2);
                cv::circle(roi,buttom,2,{0,255,0},2);
            }
            tmp_pts.emplace_back(top);
            tmp_pts.emplace_back(buttom);

        }
        if(debug_)
        {
        cv::imshow("4pts",roi);
        cv::waitKey(1);
        }

        img_pts = {
            tmp_pts[0]+origin_pt,   //tl
            tmp_pts[1]+origin_pt,   //bl
            tmp_pts[3]+origin_pt,   //br
            tmp_pts[2]+origin_pt    //tr
        };
        return true;
    }

    return false;
}



} //namespace detector