// Copyright 2017 Arunabh Sharma

#include "CheckerboardPose.h"

#include <mat.h>
#include <string>
#include <vector>

#include "SolverResectionP3P.h"
#include "glog/logging.h"
#include "opencv2/opencv.hpp"

namespace ORB_SLAM2
{
CheckerboardPose::CheckerboardPose(int chk_h, int chk_w, float l)
{
  chk_h_ = chk_h;
  chk_w_ = chk_w;
  l_     = l;
}

CheckerboardPose::~CheckerboardPose()
{
}

cv::Mat CheckerboardPose::PoseFromImage(const cv::Mat &intrinsic,
                                        const std::string &mat_file)
{
  std::vector<cv::Point2f> cor_2f;
  std::vector<cv::Point3f> cor_3f;

  LoadCheckerboardFromMatlab(mat_file, cor_2f);
  // assuming a row of points gets traversed first. TO CHECK
  for (int i = 0; i < (chk_h_ - 1); i++)
  {
    for (int j = 0; j < (chk_w_ - 1); j++)
    {
      cv::Point3f temp_3dpt;
      temp_3dpt.x = l_ * j;
      temp_3dpt.y = l_ * i;
      temp_3dpt.z = 0;
      cor_3f.push_back(temp_3dpt);
    }
  }
  cv::Mat r_vec, R;
  cv::Mat t_vec;
  cv::solvePnPRansac(cor_3f, cor_2f, intrinsic, NULL, r_vec, t_vec);
  cv::Rodrigues(r_vec, R);
  cv::Mat T = cv::Mat::eye(4, 4, CV_32FC1);

  T(cv::Range(0, 3), cv::Range(0, 3)) = R;
  T.at<float>(0, 3) = t_vec.at<float>(0, 0);
  T.at<float>(1, 3) = t_vec.at<float>(1, 0);
  T.at<float>(2, 3) = t_vec.at<float>(2, 0);

  return T.inv();  // inverse because it's pose
}

void CheckerboardPose::ReadMatDouble(const std::string file_name,
                                     const std::string var_name,
                                     std::vector<double> &v)
{
  // open MAT-file
  MATFile *pmat = matOpen(file_name.c_str(), "r");
  if (!pmat) return;

  // extract the specified variable
  mxArray *arr = matGetVariable(pmat, var_name.c_str());
  if (arr && mxIsDouble(arr) && !mxIsEmpty(arr))
  {
    // copy data
    mwSize num = mxGetNumberOfElements(arr);
    double *pr = mxGetPr(arr);
    if (pr)
    {
      v.resize(num);
      v.assign(pr, pr + num);
    }
  }

  // cleanup
  mxDestroyArray(arr);
  matClose(pmat);
}

void CheckerboardPose::ReadMatBool(const std::string file_name,
                                   const std::string var_name,
                                   bool &v)
{
  // open MAT-file
  MATFile *pmat = matOpen(file_name.c_str(), "r");
  if (!pmat) return;

  // extract the specified variable
  mxArray *arr = matGetVariable(pmat, var_name.c_str());
  if (arr && mxIsLogical(arr) && !mxIsEmpty(arr))
  {
    // copy data
    mwSize num = mxGetNumberOfElements(arr);
    bool *pr   = mxGetLogicals(arr);
    if (pr)
    {
      v = pr[0];
    }
  }

  // cleanup
  mxDestroyArray(arr);
  matClose(pmat);
}

bool CheckerboardPose::LoadCheckerboardFromMatlab(
    const std::string file_name, std::vector<cv::Point2f> &cor_2d)
{
  bool img_is_used;
  std::vector<double> cor_vec;
  ReadMatBool(file_name, "imageisUsed", img_is_used);
  if (img_is_used == false)
  {
    std::cout << "Couldn't find checkerboard points in image" << std::endl;
    return false;
  }
  else
  {
    std::cout << "Checkerboard points found";
  }
  const std::string var_name = "x_" + std::to_string(1);
  ReadMatDouble(file_name, var_name, cor_vec);

  for (int i = 0; i < (chk_h_ - 1) * (chk_w_ - 1); i++)
  {
    cv::Point2f temp_pt;
    temp_pt.x = static_cast<float>(cor_vec[2 * i + 0]);
    temp_pt.y = static_cast<float>(cor_vec[2 * i + 1]);
    cor_2d.push_back(temp_pt);
  }
}

}  // namespace ORB_SLAM2
