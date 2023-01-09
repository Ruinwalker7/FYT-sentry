#ifndef DETECTOR__ONNX_DETECTOR_HPP
#define DETECTOR__ONNX_DETECTOR_HPP

#include <string>
#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

#include "armor.hpp"

namespace detector
{
class onnxDetector
{
public:
  explicit onnxDetector(
    const std::string &model_path,
    const float &conf_thresh,
    const float &iou_thresh,
    const cv::Size &input_size);

  std::vector<Armor> Detect(cv::Mat &src);

private:
  static size_t vectorProduct(const std::vector<int64_t>& vector);

  void preprocessing(cv::Mat &image, float*& blob, std::vector<int64_t>& inputTensorShape);

  std::vector<Armor> postprocessing(const cv::Size& resizedImageShape,
                                    const cv::Size& originalImageShape,
                                    std::vector<Ort::Value>& outputTensors,
                                    const float& confThreshold, const float& iouThreshold);

  void getBestClassInfo(std::vector<float>::iterator it, const int& numClasses,
                                float& bestConf, int& bestClassId);

  void letterbox(const cv::Mat& image, cv::Mat& outImage,
                      const cv::Size& newShape,
                      const cv::Scalar& color,
                      bool auto_,
                      bool scaleFill,
                      bool scaleUp,
                      int stride);

  void scaleCoords(const cv::Size& imageShape, 
                          cv::Rect& coords,
                          std::vector<cv::Point2f>& landmarks, 
                          const cv::Size& imageOriginalShape);

private:
  Ort::Env env_;
  Ort::Session session_;
  Ort::SessionOptions session_options_;

  std::vector<char*> input_names_;
  std::vector<char*> output_names_;

  cv::Size input_size_;
  float conf_thresh_;
  float iou_thresh_;
};
} // namespace detector


#endif