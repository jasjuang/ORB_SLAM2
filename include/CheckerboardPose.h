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

  void WriteMatToFile(cv::Mat m, const char *filename);

  /**
   * @brief      Returns pose of the camera that clicked the image
   *
   * @param[in]  intrinsic Input intrinsic camera matrix
   * @param[in]  mat_file  Input name of mat file with checkerboard
   * coordinates
   *
   * @return     Output pose of camera
   */
  cv::Mat PoseFromMatlab(const cv::Mat &intrinsic, const std::string &mat_file);

  /**
   * @brief      Returns pose of the camera that clicked the image
   *
   * @param[in]  intrinsic  Input intrinsic camera matrix
   * @param[in]  reg_cor    The 2D checkerboard image coordinates
   *
   * @return     Output pose of camera
   */
  cv::Mat PoseFromCV(const cv::Mat &intrinsic,
                     const std::vector<cv::Point2f> &reg_cor);

  /**
   * @brief      Finds the scale of ORB SLAM versus the real world
   *
   * @param[in]  intrinsic  Intrinsic parameters of the camera
   * @param[in]  pose1      First pose coming out of ORB SLAM, generally
   *                        identity
   * @param[in]  pose2      Second pose coming out of the second keyframe
   * @param[in]  pts1       Vector of points of checkerboard in image1
   * @param[in]  pts2       Vector of points of checkerboard in image2
   *
   * @return     scale of ORBSLAM v/s real world
   */
  float FindScale(const cv::Mat &intrinsic,
                  const cv::Mat &pose1,
                  const cv::Mat &pose2,
                  const std::vector<cv::Point2f> &pts1,
                  const std::vector<cv::Point2f> &pts2);

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
