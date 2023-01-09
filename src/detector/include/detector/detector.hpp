#ifndef DETECTOR__DETECTOR_HPP
#define DETECTOR__DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <stdio.h>


namespace detector{

	namespace device_type
	{
		typedef enum{
			NUC,
			NX,
			OTHERS
		} device;
	}

class Detector
{
public:
	explicit Detector(
		std::vector<std::string> &files,
		const cv::Size &input_size,
		const int &device,
		const float &conf_thresh,
		const float &nms_thresh,
		const bool &is_debug);

	~Detector();
	void Read_Model_Armor();
	void Set_WH(int width, int height);
	bool Detecting(cv::Mat frame,std::vector<cv::Rect>&box,std::vector<int> &id);

	std::vector<cv::String> GetOutputsNames();
	void PostProcess();
	void Drawer();
	void DrawBoxes(int classId, float conf, int left, int top, int right, int bottom);
	
	cv::Mat GetFrame();
	int GetResWidth();
	int GetResHeight();

private:
	int m_width;
	int m_height;

	cv::dnn::Net model_armor;
	cv::Mat m_frame;
	cv::Mat m_blob;
	std::vector<cv::Mat> m_outs;
	std::vector<float> m_confs;
	std::vector<cv::Rect> m_boxes;
	std::vector<int> m_classIds;
	std::vector<int> m_perfIndx;

	int m_inpWidth;
	int m_inpHeight;
	float m_confThro;
	float m_NMSThro;
	std::vector<std::string> m_classes;
	std::vector<cv::String> names;
	std::string class_path_;
	cv::String model_path_;
	cv::String config_path_;

	int device_;
	bool debug_;
private:
	void Dump();
};


}	//namespace detector




#endif