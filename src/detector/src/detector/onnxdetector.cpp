#include "detector/onnxdetector.hpp"

namespace detector
{
onnxDetector::onnxDetector(
  const std::string &model_path,
  const float &conf_thresh,
  const float &iou_thresh,
  const cv::Size &input_size)
:session_(nullptr),conf_thresh_(conf_thresh),iou_thresh_(iou_thresh)
{
  env_ = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "YOLOv5-face");
  std::cout<<"Model Path is:"<<model_path<<std::endl;
  session_options_ = Ort::SessionOptions();
  session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
  std::vector<std::string> availableProviders = Ort::GetAvailableProviders();
  auto cudaAvailable = std::find(availableProviders.begin(), availableProviders.end(), "CUDAExecutionProvider");
  OrtCUDAProviderOptions cudaOption;
  if (cudaAvailable != availableProviders.end())
  {
      std::cout << "Inference device: GPU" << std::endl;
      session_options_.AppendExecutionProvider_CUDA(cudaOption);
  }
  else
  {
      std::cout << "Inference device: CPU" << std::endl;
  }

  session_ = Ort::Session(env_, model_path.c_str(), session_options_);

  Ort::AllocatorWithDefaultOptions allocator;

  Ort::TypeInfo input_type_info = session_.GetInputTypeInfo(0);
  std::vector<int64_t> input_tensor_shape = input_type_info.GetTensorTypeAndShapeInfo().GetShape();
  auto input_ptr = session_.GetInputNameAllocated(0,allocator);
  auto output_ptr = session_.GetOutputNameAllocated(0,allocator);
  std::cout<<output_ptr.get()<<std::endl;
  input_names_.push_back("input");
  output_names_.push_back("output");
  
  input_size_ = cv::Size2f(input_size);
}

void onnxDetector::preprocessing(cv::Mat &image, float*& blob, std::vector<int64_t>& inputTensorShape)
{
  cv::Mat resizedImage, floatImage;
  cv::cvtColor(image, resizedImage, cv::COLOR_BGR2RGB);
  //  resizedImage = image.clone();
  letterbox(resizedImage, resizedImage, this->input_size_,
                cv::Scalar(114, 114, 114), false,
                false, true, 32);

  inputTensorShape[2] = resizedImage.rows;
  inputTensorShape[3] = resizedImage.cols;

  resizedImage.convertTo(floatImage, CV_32FC3, 1 / 255.0);
  blob = new float[floatImage.cols * floatImage.rows * floatImage.channels()];
  cv::Size floatImageSize {floatImage.cols, floatImage.rows};

  // hwc -> chw
  std::vector<cv::Mat> chw(floatImage.channels());
  for (int i = 0; i < floatImage.channels(); ++i)
  {
      chw[i] = cv::Mat(floatImageSize, CV_32FC1, blob + i * floatImageSize.width * floatImageSize.height);
  }
  cv::split(floatImage, chw);
}


size_t onnxDetector::vectorProduct(const std::vector<int64_t>& vector)
{
    if (vector.empty())
        return 0;

    size_t product = 1;
    for (const auto& element : vector)
        product *= element;

    return product;
}

void onnxDetector::letterbox(const cv::Mat& image, cv::Mat& outImage,
                      const cv::Size& newShape = cv::Size(640, 640),
                      const cv::Scalar& color = cv::Scalar(114, 114, 114),
                      bool auto_ = true,
                      bool scaleFill = false,
                      bool scaleUp = true,
                      int stride = 32)
{
  cv::Size shape = image.size();
    float r = std::min((float)newShape.height / (float)shape.height,
                       (float)newShape.width / (float)shape.width);
    if (!scaleUp)
        r = std::min(r, 1.0f);

    float ratio[2] {r, r};
    int newUnpad[2] {(int)std::round((float)shape.width * r),
                     (int)std::round((float)shape.height * r)};

    auto dw = (float)(newShape.width - newUnpad[0]);
    auto dh = (float)(newShape.height - newUnpad[1]);

    if (auto_)
    {
        dw = (float)((int)dw % stride);
        dh = (float)((int)dh % stride);
    }
    else if (scaleFill)
    {
        dw = 0.0f;
        dh = 0.0f;
        newUnpad[0] = newShape.width;
        newUnpad[1] = newShape.height;
        ratio[0] = (float)newShape.width / (float)shape.width;
        ratio[1] = (float)newShape.height / (float)shape.height;
    }

    dw /= 2.0f;
    dh /= 2.0f;

    if (shape.width != newUnpad[0] && shape.height != newUnpad[1])
    {
        cv::resize(image, outImage, cv::Size(newUnpad[0], newUnpad[1]));
    }

    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
}

void onnxDetector::scaleCoords(const cv::Size& imageShape, cv::Rect& coords, std::vector<cv::Point2f>& landmarks, const cv::Size& imageOriginalShape)
{
  float gain = std::min((float)imageShape.height / (float)imageOriginalShape.height,
                        (float)imageShape.width / (float)imageOriginalShape.width);

  int pad[2] = {(int) (( (float)imageShape.width - (float)imageOriginalShape.width * gain) / 2.0f),
                (int) (( (float)imageShape.height - (float)imageOriginalShape.height * gain) / 2.0f)};

  coords.x = (int) std::round(((float)(coords.x - pad[0]) / gain));
  coords.y = (int) std::round(((float)(coords.y - pad[1]) / gain));

  coords.width = (int) std::round(((float)coords.width / gain));
  coords.height = (int) std::round(((float)coords.height / gain));

  landmarks[0].x = (int) std::round(((float)(landmarks[0].x - pad[0]) / gain));
  landmarks[0].y = (int) std::round(((float)(landmarks[0].y - pad[1]) / gain));
  landmarks[1].x = (int) std::round(((float)(landmarks[1].x - pad[0]) / gain));
  landmarks[1].y = (int) std::round(((float)(landmarks[1].y - pad[1]) / gain));
  landmarks[2].x = (int) std::round(((float)(landmarks[2].x - pad[0]) / gain));
  landmarks[2].y = (int) std::round(((float)(landmarks[2].y - pad[1]) / gain));
  landmarks[3].x = (int) std::round(((float)(landmarks[3].x - pad[0]) / gain));
  landmarks[3].y = (int) std::round(((float)(landmarks[3].y - pad[1]) / gain));
  landmarks[4].x = (int) std::round(((float)(landmarks[4].x - pad[0]) / gain));
  landmarks[4].y = (int) std::round(((float)(landmarks[4].y - pad[1]) / gain));
}


std::vector<Armor> onnxDetector::Detect(cv::Mat &src)
{
  cv::Mat tmp = src.clone();
  float *blob = nullptr;
  std::vector<int64_t> inputTensorShape {1, 3, -1, -1};
  this->preprocessing(src, blob, inputTensorShape);

  size_t inputTensorSize = vectorProduct(inputTensorShape);

  std::vector<float> inputTensorValues(blob, blob + inputTensorSize);

  std::vector<Ort::Value> inputTensors;

  Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
          OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

  inputTensors.push_back(Ort::Value::CreateTensor<float>(
          memoryInfo, inputTensorValues.data(), inputTensorSize,
          inputTensorShape.data(), inputTensorShape.size()
  ));
  int64_t start_time = (double)cv::getTickCount();
  Ort::RunOptions run_options;
  std::vector<Ort::Value> outputTensors = this->session_.Run(Ort::RunOptions{nullptr},
                                                            input_names_.data(),
                                                            inputTensors.data(),
                                                            1,
                                                            output_names_.data(),
                                                            1);

  int64_t end_time = (double)cv::getTickCount();
  double dt = (end_time - start_time)/(double)cv::getTickFrequency();
  std::cout<<"dt="<<dt<<std::endl;
  cv::Size resizedShape = cv::Size((int)inputTensorShape[3], (int)inputTensorShape[2]);
  std::vector<Armor> result = this->postprocessing(resizedShape,
                                                        src.size(),
                                                        outputTensors,
                                                        conf_thresh_, iou_thresh_);
  

  // cv::Mat tmp = cv::Mat::zeros(cv::Size(1280,720),CV_8UC3);
  // std::cout<<"result_size="<<result[0].box.tl()<<std::endl;
  for(auto r : result)
  {
    cv::rectangle(tmp,r.box,cv::Scalar(0,255,0),3);
    cv::putText(tmp,"id:"+std::to_string(r.id),r.box.tl(),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(0,255,255));
    for(int i =0;i<4;i++)
    {
      auto p = r.points[i];
      cv::circle(tmp,p,2,{0,0,255},2);
    }
  }
  cv::putText(tmp,"fps:"+std::to_string(1.0/dt),{50,100},cv::FONT_HERSHEY_SIMPLEX,1,{255,255,255});
  cv::imshow("s",tmp);
  cv::waitKey(1);
  // std::cout<<"result_size="<<result[0].points[0].x<<std::endl;

  // std::cout<<"result_size="<<result[0].points[0].x<<std::endl;

  delete[] blob;

  return result;
}

void onnxDetector::getBestClassInfo(std::vector<float>::iterator it, const int& numClasses,
                                    float& bestConf, int& bestClassId)
{
    // first 15 element are box and obj confidence and landmarks
    bestClassId = 15;
    bestConf = 0;

    for (int i = 15; i < numClasses + 15; i++)
    {
        if (it[i] > bestConf)
        {
            bestConf = it[i];
            bestClassId = i - 15;
        }
    }

}

std::vector<Armor> onnxDetector::postprocessing(const cv::Size& resizedImageShape,
                                  const cv::Size& originalImageShape,
                                  std::vector<Ort::Value>& outputTensors,
                                  const float& confThreshold, const float& iouThreshold)
{
  std::vector<cv::Rect> boxes;
  std::vector<float> confs;
  std::vector<int> classIds;
  std::vector<std::vector<cv::Point2f>> landmarks;

  auto* rawOutput = outputTensors[0].GetTensorData<float>();
  std::vector<int64_t> outputShape = outputTensors[0].GetTensorTypeAndShapeInfo().GetShape();
  size_t count = outputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
  std::vector<float> output(rawOutput, rawOutput + count);

  // for (const int64_t& shape : outputShape)
  //     std::cout << "Output Shape: " << shape << std::endl;

  // 16 elements : xmin,ymin,xamx,ymax,box_score,x1,y1, ... ,x5,y5,face_score
  int numClasses = (int)outputShape[2] - 16;
  int elementsInBatch = (int)(outputShape[1] * outputShape[2]);

  // only for batch size = 1
  for (auto it = output.begin(); it != output.begin() + elementsInBatch; it += outputShape[2])
  {
    float objConf = it[4];

    if (objConf > confThreshold)
    {
      int centerX = (int) (it[0]);
      int centerY = (int) (it[1]);
      int width = (int) (it[2]);
      int height = (int) (it[3]);
      int left = centerX - width / 2;
      int top = centerY - height / 2;
      cv::Point2f landmark1{it[5],it[6]};
      cv::Point2f landmark2{it[7],it[8]};
      cv::Point2f landmark3{it[9],it[10]};
      cv::Point2f landmark4{it[11],it[12]};
      cv::Point2f landmark5{it[13],it[14]};

      float clsConf;
      int classId;
      this->getBestClassInfo(it, numClasses, clsConf, classId);

      float confidence = clsConf * objConf;

      boxes.emplace_back(left, top, width, height);
      confs.emplace_back(confidence);
      classIds.emplace_back(classId);
      landmarks.emplace_back(std::vector<cv::Point2f>{landmark1,landmark2,landmark3,landmark4,landmark5});
    }
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confs, confThreshold, iouThreshold, indices);
  // std::cout << "amount of NMS indices: " << indices.size() << std::endl;

  std::vector<Armor> detections;

  for (int idx : indices)
  {
      Armor det;
      det.box = cv::Rect(boxes[idx]);
      det.points = landmarks[idx];
      scaleCoords(resizedImageShape, det.box, det.points, originalImageShape);

      det.id = classIds[idx];
      detections.emplace_back(det);
  }


  return detections;
}

} //namespace detector

