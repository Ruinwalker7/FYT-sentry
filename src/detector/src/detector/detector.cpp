#include "detector/detector.hpp"

namespace detector{

Detector::Detector(
  std::vector<std::string> &files,
  const cv::Size &input_size,
  const int &device,
  const float &conf_thresh,
  const float &nms_thresh,
  const bool &is_debug)
: device_(device),
  debug_(is_debug),
  m_confThro(conf_thresh),
  m_NMSThro(nms_thresh),
  m_inpHeight(input_size.height),
  m_inpWidth(input_size.width)
{
  class_path_ = files[0];
  model_path_ = files[1];
  config_path_ = files[2];
	Read_Model_Armor();
}

Detector::~Detector()
{
  Dump();
}

void Detector::Dump()
{	
	m_outs.clear();
	m_boxes.clear();
	m_confs.clear();
	m_classIds.clear();
	m_perfIndx.clear();
}

void Detector::Read_Model_Armor()
{	      
	std::ifstream ifs(class_path_.c_str());
	std::string line;
	while (getline(ifs, line)) m_classes.push_back(line);

	model_armor = cv::dnn::readNetFromDarknet(config_path_, model_path_);


	if(device_ == device_type::NX)
  {
		printf("nx\n");
    model_armor.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    model_armor.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
  }
	else if(device_ == device_type::NUC)
  {
    model_armor.setPreferableBackend(cv::dnn::DNN_BACKEND_INFERENCE_ENGINE);
    model_armor.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);
  }
  else
  {
    model_armor.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    model_armor.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  }
	
}

bool Detector::Detecting(cv::Mat frame,std::vector<cv::Rect>&box,std::vector<int> &id)
{

  timespec t0, t_final;
  double t;
  if(debug_){
      clock_gettime(CLOCK_MONOTONIC,&t0);
      t = (double)cv::getTickCount();
  }


	m_width=frame.cols;
	m_height=frame.rows;

    m_frame = frame.clone();
	cv::dnn::blobFromImage(m_frame, m_blob, 1 / 255.0,cv::Size(m_inpWidth, m_inpHeight), cv::Scalar(0, 0, 0), true, false);

	model_armor.setInput(m_blob);


	model_armor.forward(m_outs, GetOutputsNames());


	PostProcess();
		
	if(debug_){
		t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
		Drawer();
		cv::namedWindow("CNN", cv::WINDOW_AUTOSIZE);
		cv::putText(m_frame, "fps "+std::to_string(1.0/t), cv::Point(20,50),cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 0));
		cv::imshow("CNN", m_frame);
		cv::waitKey(1);
	}


  if(m_perfIndx.size()==0) return false;
        
  for(int i=0;i<m_perfIndx.size();i++)
  {
    id.push_back(m_classIds[m_perfIndx[i]]);
    box.push_back(m_boxes[m_perfIndx[i]]);
  }

  Dump();
	return true;
}

std::vector<cv::String> Detector::GetOutputsNames()
{
	static std::vector<cv::String> names;
	if (names.empty())
	{
		std::vector<int> outLayers = model_armor.getUnconnectedOutLayers();
		std::vector<cv::String> layersNames = model_armor.getLayerNames();

		names.resize(outLayers.size());
		for (int i = 0; i < outLayers.size(); ++i)
			names[i] = layersNames[outLayers[i] - 1];
	}
	return names;
}


void Detector::PostProcess()
{
	for (int num = 0; num < m_outs.size(); num++)
	{
		cv::Point Position;
		double confidence;

		float* data = (float*)m_outs[num].data;
		for (int j = 0; j < m_outs[num].rows; j++, data += m_outs[num].cols)
		{
			cv::Mat scores = m_outs[num].row(j).colRange(5, m_outs[num].cols);
			minMaxLoc(scores, 0, &confidence, 0, &Position);
			
			if (confidence > m_confThro)
			{
				int centerX = (int)(data[0] * m_width);
				int centerY = (int)(data[1] * m_height);
				int width = (int)(data[2] * m_width);
				int height = (int)(data[3] * m_height);
				int left = centerX - width / 2;
				int top = centerY - height / 2;
				
				m_classIds.push_back(Position.x);
				m_confs.push_back((float)confidence);
				m_boxes.push_back(cv::Rect(left, top, width, height));
			}
		}
	}
	cv::dnn::NMSBoxes(m_boxes, m_confs, m_confThro, m_NMSThro, m_perfIndx);
}

void Detector::Drawer()
{
	for (int i = 0; i < m_perfIndx.size(); i++)
	{
		int idx = m_perfIndx[i];
		cv::Rect box = m_boxes[idx];
		DrawBoxes(m_classIds[idx], m_confs[idx], box.x, box.y,box.x + box.width, box.y + box.height);					
	}
}

void Detector::DrawBoxes(int classId, float conf, int left, int top, int right, int bottom)
{
	cv::rectangle(m_frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);

	std::string label = cv::format("%.2f", conf);
	if (!m_classes.empty())
	{
		CV_Assert(classId < (int)m_classes.size());
		label = m_classes[classId] + ":" + label;
	}

	int baseLine;
	cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
	top = std::max(top, labelSize.height);
	cv::rectangle(m_frame, cv::Point(left, top - round(1.5*labelSize.height)), cv::Point(left + round(1.5*labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
	cv::putText(m_frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);
}

cv::Mat Detector::GetFrame()
{
	return m_frame;
}

int Detector::GetResWidth()
{
	return m_width;
}

int Detector::GetResHeight()
{
	return m_height;
}

} //namespace detector