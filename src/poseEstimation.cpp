#include "poseEstimation.h"

poseEstimation::poseEstimation()
{
    std::cout<< "created Pose Estimator"<<std::endl;
}
/*
Initialize class members with required parameters:
    1. bordSize - defines size of checkerboard
    2. squareSize - defines size checkerboard square in m
    3. cameraMatrix - opencv 4x4 camera matrix 
    4. distCoeffs - distortion vector
*/
void poseEstimation::initialize(cv::Size_<int> boardSize,float squareSize,const cv::Mat& cameraMatrix,const std::vector<double>& distCoeffs)
{   
    boardSize_ = boardSize;
    squareSize_ = squareSize;
    cameraMatrix_ = cameraMatrix;
    distCoeffs_ = distCoeffs;
    //create 3D chessboard corner points in chessoard reference frame
    calcChessboardCorners_();
    //create coordinate axes
    axis_.push_back(cv::Point3d(3 * squareSize_, 0.0, 0.0));
    axis_.push_back(cv::Point3d(0.0, 3 * squareSize_, 0.0));
    axis_.push_back(cv::Point3d(0.0, 0.0, 3 * squareSize_));
    std::cout<<"Chessboard Size: "<<boardSize_.width << "x"<<boardSize_.height<<std::endl;
}
poseEstimation::~poseEstimation()
{
}
/*
Draw Axes on Chessboard corner points
*/
void poseEstimation::drawAxes_(const cv::Mat &src_, cv::Mat &dst_,
                          std::vector<cv::Point2d> &imgPts_,
                          std::vector<cv::Point2f> &cornersSP_) {
  src_.copyTo(dst_);
  cv::arrowedLine(dst_, cornersSP_[0], imgPts_[0], cv::Scalar(0, 0, 255), 2,
                  cv::LINE_AA, 0);
  cv::arrowedLine(dst_, cornersSP_[0], imgPts_[1], cv::Scalar(0, 255, 0), 2,
                  cv::LINE_AA, 0);
  cv::arrowedLine(dst_, cornersSP_[0], imgPts_[2], cv::Scalar(255, 0, 0), 2,
                  cv::LINE_AA, 0);
}
/*
Computes rotation (rvec) and translation (tvec) vectors from chessboard 
    corners given in 3D chessboard frame and image corners (2D). 
*/
void poseEstimation::helper_RT_ImgPoints_(cv::Mat &src_, cv::Mat &rvec_,
                                     cv::Mat &tvec_,
                                     std::vector<cv::Point2f> &corners_,
                                     std::vector<cv::Point3f> &boardPts_) {
  cv::TermCriteria termcrit(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,
                            30, 0.001);
  cv::cornerSubPix(src_, corners_, cv::Size(11, 11), cv::Size(-1, -1),
                   termcrit);
  /*cv::solvePnPRansac(boardPts_, corners_, cameraMatrix_,
                     distCoeffs_, rvec_, tvec_, false, 100, 8.0, 0.99,
                     cv::noArray(), cv::SOLVEPNP_EPNP);*/
    cv::solvePnP(boardPts_, corners_, cameraMatrix_,
                     distCoeffs_, rvec_, tvec_, false,cv::SOLVEPNP_IPPE);
}
/*
    Transfromation from cv::Mat1f to cv::Vec3f. 
    Returns cv::Vec3f.
*/
cv::Vec3f poseEstimation::to_vec3f_(cv::Mat1f const& m)
{
    CV_Assert((m.rows == 3) && (m.cols == 1));
    return cv::Vec3f(m.at<float>(0), m.at<float>(1), m.at<float>(2));
}
/*
Compute Pose from image that contains a chessboard.
Returns true if pose could be computed. 
*/
bool poseEstimation::estimatePose(const cv::Mat &img)
{       

    cv::Mat gray, rvec, tvec, outputFrame, rotationMat;
    
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point2d> imgPts;
    
    
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    assert(!gray.empty());
    bool patternWasFound = cv::findChessboardCorners(
      gray, boardSize_,  corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
    
    if(patternWasFound)
    {
        helper_RT_ImgPoints_(gray, rvec, tvec, corners, objectPoints_);
        

        cv::projectPoints(axis_, rvec, tvec, this->cameraMatrix_, this->distCoeffs_,
                        imgPts, cv::noArray(), 0);
        drawAxes_(img, outputFrame, imgPts, corners);
     
        cv::drawChessboardCorners(outputFrame, boardSize_, corners,patternWasFound);
    
        RVec_ =  to_vec3f_(rvec);
        cv::Vec3f translationVec =  to_vec3f_(tvec);
        
        H_ = cv::Affine3f(RVec_, translationVec);

        return true;
    }
    else{
        std::cout<<"No corners could be found"<<std::endl;
        return 0;
    }
    
}
cv::Affine3<float> poseEstimation::getTransform()
{
    return H_;
}
cv::Vec3f poseEstimation::getRVec()
{
    return RVec_;
}
/*
Create 3D chessboard corner points. 
*/
void poseEstimation::calcChessboardCorners_()
{
        objectPoints_.resize(0);
        
        
        for( int i = 0; i < 5; i++ )
        {
            for( int j = 0; j < 8; j++ )
            {
                objectPoints_.push_back(cv::Point3f(float(j*squareSize_),
                                          float(i*squareSize_), 0));
            }    
        } 
}
