// Copyright 2017 Arunabh Sharma

#ifndef INCLUDE_CHECKERBOARDPOSE_H_
#define INCLUDE_CHECKERBOARDPOSE_H_

#include <string>

#include "opencv2/opencv.hpp"

namespace ORB_SLAM2
{
/**
 * @brief      Class that calculates the pose using a P3P solver in an image
 *             with a known checkerboard target.
 * @author     Arunabh Sharma
 * @ingroup    ORB_SLAM2
 * @attention  This file is free to use with appropriate citations of the
 *             author.
 */
class CheckerboardPose
{
 public:
  CheckerboardPose(int chk_h, int chk_w, float l);
  ~CheckerboardPose();
  /**
   * @brief      Returns pose of the camera that clicked the image
   *
   * @param[in]  intrinsic Input intrinsic camera matrix
   * @param[in]  mat_file  Input name of mat file with checkerboard
   * coordinates
   *
   * @return     Output pose of camera
   */
  cv::Mat PoseFromImage(const cv::Mat &intrinsic, const std::string &mat_file);

 private:
  bool LoadCheckerboardFromMatlab(const std::string file_name,
                                  std::vector<cv::Point2f> &cor_2d);

  void ReadMatBool(const std::string file_name,
                   const std::string var_name,
                   bool &v);

  void ReadMatDouble(const std::string file_name,
                     const std::string var_name,
                     std::vector<double> &v);

  int chk_h_ = 0;  ///< Number of checkerboard squares vertically not vertices
  int chk_w_ = 0;  ///< Number of checkerboard squares horizontally not vertices
  float l_   = 0;  ///< Length of each square in metres
};
}  // namespace ORB_SLAM2

#endif  // INCLUDE_CHECKERBOARDPOSE_H_
