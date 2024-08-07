#ifndef INCLUDE_POSEESTIMATION_H_
#define INCLUDE_POSEESTIMATION_H_
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

/* poseEstimation class

 This class serves as a wrapper for computing an external 
 camera pose relative to a checkerboard. Under the hood is OpenCV's solvePnp function that solves 
 the pose computation problem by solving for the rotation and translation 
 that minimizes the reprojection error from 3D-2D point correspondences.
*/
class poseEstimation
{
private:
  cv::Vec3f RVec_;
  cv::Mat cameraMatrix_; 
  std::vector<double> distCoeffs_;
  cv::Size_<int> patternSize_;
  std::vector<cv::Point3f> objectPoints_;
  std::vector<cv::Point3d> axis_;
  cv::Size_<int> boardSize_;
  float squareSize_;
  cv::Vec3f to_vec3f_(cv::Mat1f const& m);
  cv::Affine3<float> H_;
  void calcChessboardCorners_();
    void drawAxes_(const cv::Mat &src_, cv::Mat &dst_,
                          std::vector<cv::Point2d> &imgPts_,
                          std::vector<cv::Point2f> &cornersSP_);
    void helper_RT_ImgPoints_(cv::Mat &src_, cv::Mat &rvec_,
                                     cv::Mat &tvec_,
                                     std::vector<cv::Point2f> &corners_,
                                     std::vector<cv::Point3f> &boardPts_);
public:
    
    poseEstimation();
    void initialize(cv::Size_<int> boardSize,float squareSize,const cv::Mat& cameraMatrix,const  std::vector<double>& distCoeffs);
    bool estimatePose(const cv::Mat &img);
    cv::Affine3<float> getTransform();
    cv::Vec3f getRVec();
    ~poseEstimation();
};





#endif