/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
* of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <string>
#include "boost/algorithm/string.hpp"

#include "System.h"

using namespace std;

int main(int argc, char **argv)
{
  argc = argc;
  /////////////////////////////////////////////////////////////////////
  ////////////////////// Variable Instantiation////////////////////////
  /////////////////////////////////////////////////////////////////////

  std::string yamlfile =
      "/home/hypevr/hvr-lidar/apps/PoseGraphOptimization/Data/ORBLidar.yaml";
  std::string camtxtfile = "/media/hypevr/HDD/Data_M3/cam_timestamp_walk1.txt";
  std::string vocfile =
      "/home/hypevr/hvr-lidar/apps/PoseGraphOptimization/Data/"
      "ORBvoc.txt";
  std::ifstream cam_fs;
  std::string in_line;

  cam_fs.open(camtxtfile, std::ifstream::in);
  std::vector<std::pair<int, int>> loop_closure_vec_inst;
  std::vector<std::string> loop_str_vec_inst;
  std::vector<std::string> img_name;
  std::vector<int64_t> timestamp_vec;

  cv::Mat intrinsic = cv::Mat::eye(3, 3, CV_32F);
  // std::vector<cv::Mat> img_vec;
  std::vector<std::string> img_vec;
  std::vector<cv::Mat> Todom_cv_vec;
  std::vector<cv::Mat> Todom_cv_vec_bundle;
  std::vector<cv::Mat> Tmap_cv_vec;
  std::vector<cv::Mat> Tmap1_cv_vec;
  // intrinsic.at<float>(0, 0) = 1492.2556066755144f;
  // intrinsic.at<float>(1, 1) = 1492.2556066755144f;
  // intrinsic.at<float>(0, 2) = 2538.37954716574f;
  // intrinsic.at<float>(1, 2) = 1450.0549836353156f;
  intrinsic.at<float>(0, 0) = 746.127803338f;
  intrinsic.at<float>(1, 1) = 746.127803338f;
  intrinsic.at<float>(0, 2) = 1269.189773583f;
  intrinsic.at<float>(1, 2) = 725.027491818f;
  int image_counter = 0;
  ///////////////////////////////////////////////////////////////////////
  //////////////////////// Reading Files/////////////////////////////////
  ///////////////////////////////////////////////////////////////////////

  std::getline(cam_fs, in_line);

  std::size_t *pos = 0;
  int base         = 10;
  while (in_line != "")
  {
    if (image_counter % 5 == 0)
    {
      boost::split(
          img_name, in_line, boost::is_any_of("\t"), boost::token_compress_on);
      // cv::Mat img_temp  = cv::imread(img_name[0]);
      int64_t time_inst = std::stoull(img_name[1], pos, base);
      // cv::Mat out_img;
      // cv::resize(img_temp, out_img, cv::Size(), 0.5, 0.5);
      // img_vec.push_back(out_img);
      img_vec.push_back(img_name[0]);
      timestamp_vec.push_back(time_inst);
      std::getline(cam_fs, in_line);
    }
    else
    {
      std::getline(cam_fs, in_line);
    }
    image_counter++;
  }
  ORB_SLAM2::System SLAM(vocfile, yamlfile, ORB_SLAM2::System::MONOCULAR, true);
  // Pass the image to the SLAM system
  for (std::size_t i = 0; i < img_vec.size(); i++)
  {
    std::cout << "iteration " << i << std::endl;
    cv::Mat image_temp = cv::imread(img_vec[i]);
    cv::Mat out_img;
    cv::resize(image_temp, out_img, cv::Size(), 0.5, 0.5);
    // LOG(INFO) << "Iteration" << i << std::endl;
    cv::Mat T_temp = cv::Mat::eye(4, 4, CV_32FC1);
    T_temp = SLAM.TrackMonocular(out_img, static_cast<double>(i * 0.01));
    std::cout << "Matrix " << T_temp << std::endl;
  }

  std::chrono::milliseconds timespan(12200000);
  std::this_thread::sleep_for(timespan);

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return 0;
}
