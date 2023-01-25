#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/String.h"
#include "camera/GxIAPI.h"
#include "camera/DxImageProc.h"
#include "camera/CircularQueue.h"
#include "sensor_msgs/CameraInfo.h"


#define BYTE unsigned char



GX_DEV_HANDLE       m_hDevice;              ///< 设备句柄  
unsigned char       *m_pBufferRaw;          ///< 原始图像数据  
unsigned char       *m_pBufferRGB;          ///< RGB图像数据，用于显示和保存bmp图像  
// int64_t           m_nImageHeight;         ///< 原始图像高  
// int64_t           m_nImageWidth;          ///< 原始图像宽  
int64_t             m_nPayLoadSize;
int64_t             m_nPixelColorFilter;    ///< Bayer格式  

cv::Mat camera_img;
sensor_msgs::Image img;
image_transport::Publisher pub_image1,pub_image2,pub_image3,pub_image4;
ros::Subscriber sub_cameraParam;
static int resolution_width;
static int resolution_height;
static int auto_white_balance;
static int frame_rate;
static int auto_explosure;
static int explosure_time;
static int init_flag=0;


//图像回调处理函数 
static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{ 
	if (pFrame->status == 0)
	{
		//对图像进行某些操作  
		memcpy(m_pBufferRaw, pFrame->pImgBuf, pFrame->nImgSize);
		
		// RGB转换  
		DxRaw8toRGB24(m_pBufferRaw
			, m_pBufferRGB
			, (VxUint32)(resolution_width)
			, (VxUint32)(resolution_height)
			, RAW2RGB_NEIGHBOUR
			, DX_PIXEL_COLOR_FILTER(m_nPixelColorFilter)
			, false); 

		//cv方法传输图像
		memcpy(camera_img.data, m_pBufferRGB, resolution_width*resolution_height * 3);
		std_msgs::Header hd;
		hd.stamp = ros::Time::now();
		pub_image1.publish(cv_bridge::CvImage(hd, "rgb8", camera_img).toImageMsg());
	}
}


bool init(){
	GX_STATUS emStatus = GX_STATUS_SUCCESS;
	GX_OPEN_PARAM openParam;
	uint32_t      nDeviceNum = 0;
	openParam.accessMode = GX_ACCESS_EXCLUSIVE;
	openParam.openMode = GX_OPEN_INDEX;
	openParam.pszContent = "1";
	// 初始化库 
	emStatus = GXInitLib();
	if (emStatus != GX_STATUS_SUCCESS)
	{
		printf("Can't init lib.\n");
		return 0;
	}
	// 枚举设备列表  
	emStatus = GXUpdateDeviceList(&nDeviceNum, 1000);
	if ((emStatus != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
	{
		printf("Can't find camera.\n");
		return 0;
	}
	//打开设备  
	emStatus = GXOpenDevice(&openParam, &m_hDevice);
	if (emStatus != GX_STATUS_SUCCESS)
	{
		printf("Camera open fail\n");
		return 0;
	}	
	
	// // 获取宽度  
	// emStatus = GXGetInt(m_hDevice, GX_INT_WIDTH, &m_nImageWidth);
	// // 获取高度  
	// emStatus = GXGetInt(m_hDevice, GX_INT_HEIGHT, &m_nImageHeight);

	// 设置宽度  
	emStatus = GXSetInt(m_hDevice, GX_INT_WIDTH, resolution_width);
	// 设置高度  
	emStatus = GXSetInt(m_hDevice, GX_INT_HEIGHT, resolution_height);

	// 从中心裁剪
	int64_t nOffsetX = (1280 - resolution_width)/2;
	int64_t nOffsetY = (1024 - resolution_height)/2;
	GXSetEnum(m_hDevice, GX_ENUM_RREGION_SELECTOR, GX_REGION_SELECTOR_REGION0);
	GXSetInt(m_hDevice, GX_INT_OFFSET_X, nOffsetX);
	GXSetInt(m_hDevice, GX_INT_OFFSET_Y, nOffsetY);

	// 获取图像大小  
	emStatus = GXGetInt(m_hDevice, GX_INT_PAYLOAD_SIZE, &m_nPayLoadSize);
	//设置采集模式连续采集  
	emStatus = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
	emStatus = GXSetInt(m_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 1);
	
	bool bColorFliter = false;

	//设置图像大小
	camera_img.create(resolution_height, resolution_width, CV_8UC3);

	//设置是否开启自动白平衡
	emStatus=GXSetEnum(m_hDevice,GX_ENUM_BALANCE_WHITE_AUTO,auto_white_balance);
	//设置白平衡数值  如果开启自动白平衡则无效
	GXSetEnum(m_hDevice,GX_ENUM_LIGHT_SOURCE_PRESET,GX_LIGHT_SOURCE_PRESET_DAYLIGHT_5000K);

	GXSetEnum(m_hDevice, GX_ENUM_AWB_LAMP_HOUSE,0);

	
	//设置帧率
	GXSetFloat(m_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, frame_rate);

	//设置曝光时间
	GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, explosure_time);
	
	//设置彩色图像
	emStatus = GXGetEnum(m_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &m_nPixelColorFilter);


	// m_pBufferRGB = new unsigned char [(size_t)(m_nImageWidth * m_nImageHeight * 3)];
	m_pBufferRGB = new unsigned char [(size_t)(resolution_width * resolution_height * 3)];

	if (m_pBufferRGB == NULL)
	{
		return false;
	}
	//为存储原始图像数据申请空间  
	m_pBufferRaw = new unsigned char [(size_t)m_nPayLoadSize];
	if (m_pBufferRaw == NULL)
	{
		delete[]m_pBufferRGB;
		m_pBufferRGB = NULL;
		return false;
	}
	//注册图像处理回调函数 
	std::cout<<"ready open daheng"<<std::endl; 
	init_flag=1;
	emStatus = GXRegisterCaptureCallback(m_hDevice, NULL, OnFrameCallbackFun);

	//声明image消息内容,不通过cv时取消注释
	img.header = std_msgs::Header();
	img.encoding = "rgb8";
	img.height = resolution_height;
	img.width = resolution_width;
	img.step = resolution_width*3;

	//发送开采命令  
	emStatus = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
	//---------------------  
	//在这个区间图像会通过OnFrameCallbackFun接口返给用户 
	std::cout<<"输入ctrl+c关闭程序"<<std::endl;

	cv::VideoCapture capture1(0, cv::CAP_V4L);
	capture1.set(cv::CAP_PROP_FRAME_HEIGHT,800);
    capture1.set(cv::CAP_PROP_FRAME_WIDTH,600);
    capture1.set(cv::CAP_PROP_FPS,30);
	// capture1.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);//亮度 1

    capture1.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('M','J','P','G'));
    capture1.open(-1);
    if(!capture1.isOpened()){
            ROS_INFO("Camera2 Error!");
    }else{
		ROS_INFO("Camera2 opened");
	}
	
	cv::Mat camera_img1;

	while(ros::ok()){
		capture1.set(cv::CAP_PROP_EXPOSURE,-4);
		GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, explosure_time);
		GXSetEnum(m_hDevice,GX_ENUM_BALANCE_WHITE_AUTO,auto_white_balance);
		capture1>>camera_img1;
				std_msgs::Header hd;
		hd.stamp = ros::Time::now();
		pub_image2.publish(cv_bridge::CvImage(hd, "rgb8", camera_img1).toImageMsg());
		// cv::imshow("camera2",camera_img1);
		cv::waitKey(10);
	}

	//---------------------  
	//发送停采命令  
	emStatus = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	//注销采集回调  
	emStatus = GXUnregisterCaptureCallback(m_hDevice);
	if (m_pBufferRGB != NULL)
	{
		delete[]m_pBufferRGB;
		m_pBufferRGB = NULL;
	}
	if (m_pBufferRaw != NULL)
	{
		delete[]m_pBufferRaw;
		m_pBufferRaw = NULL;
	}
	emStatus = GXCloseDevice(m_hDevice);
	emStatus = GXCloseLib();
	return 0;
}

int main(int argc,char **argv){

  ros::init(argc,argv,"camera_node");
  ros::NodeHandle n;
  n.param("/camera/resolution_width",resolution_width,800);
  n.param("/camera/resolution_height",resolution_height,600);
  n.param("/camera/auto_white_balance",auto_white_balance,1);
  n.param("/camera/frame_rate",frame_rate,210);
  n.param("/camera/explosure_time",explosure_time,20000);
  GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, explosure_time);
  GXSetEnum(m_hDevice,GX_ENUM_BALANCE_WHITE_AUTO,auto_white_balance);
  cv::Mat src;

  image_transport::ImageTransport it(n);
  pub_image1 = it.advertise("raw_img", 1);
  pub_image2 = it.advertise("raw_img2", 1);
  pub_image3 = it.advertise("raw_img3", 1);
  sensor_msgs::ImagePtr img_msg;

  while(ros::ok()){
    ros::spinOnce();
    while(init_flag==0)
    {
      init();
      if(init_flag==0){
        printf("Please reinsert the device\n");
        printf("### Error, start again! ###\n\n");}
        cv::waitKey(2000);
    };
  }
}
