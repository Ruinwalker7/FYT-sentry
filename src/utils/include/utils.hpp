#ifndef UTILS__UTILS_HPP
#define UTILS__UTILS_HPP

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "rm_interfaces/Armor.h"

namespace device_type
{
  typedef enum{
    NUC,
    NX,
    OTHERS
  } device;
}


namespace utils{

template<typename T,typename EigenT>
void toEigenMatrix(EigenT &matrix,const std::vector<T> &vector)
{

  int cnt = 0;
  for(int row = 0;row < matrix.rows();row++)
  {
    for(int col = 0;col < matrix.cols();col++)
    {
      matrix(row,col) = vector[cnt];
      cnt++;
    }
  }
}

template<typename T>
void toCvMatrix(cv::Mat& matrix,const std::vector<T>& vector)
{
  int cnt = 0;
  for(int row = 0;row < matrix.rows;row++)
  {
    for(int col = 0;col < matrix.cols;col++)
    {
      matrix.at<T>(row,col) = vector[cnt];
      cnt++;
    }
  }
}

} //namespace utils


#endif