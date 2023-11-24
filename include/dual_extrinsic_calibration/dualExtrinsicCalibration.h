#ifndef INCLUDE_DUALEXTRINSICCALIBRATION_H_
#define INCLUDE_DUALEXTRINSICCALIBRATION_H_
#include"poseEstimation.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <opencv2/core/quaternion.hpp>
#include <geometry_msgs/PoseStamped.h>


template <typename DynamicEigenMatrix>
void push_back(DynamicEigenMatrix& m, Eigen::Vector4d& values, std::size_t row)
{
    if(row >= m.rows()) {
        m.conservativeResize(row + 1, Eigen::NoChange);
    }
    m.row(row) = values;
}

class dualExtrinsicCalibration
{



private:
    poseEstimation pose1_;
    poseEstimation pose2_;
    void getVecFromMat(std::vector<double> &vec, cv::Mat &M);
    bool cvQuatToVector4d(const cv::Quat<double> &Q, Eigen::Vector4d &q);
    Eigen::MatrixXd matxd_;
    cv::Vec3d trans_acc_;
    int num_samples_, counter_;
    int sample_;
    Eigen::Vector4d avg_quat_;
    cv::Vec3d avg_trans_;
public:
    dualExtrinsicCalibration();
    dualExtrinsicCalibration(cv::Size_<int> boardSize,float squareSize, std::string calib_yaml_1, std::string calib_yaml_2);
    dualExtrinsicCalibration(cv::Size boardSize,float squareSize,
                            cv::Mat& cameraMatrix1, std::vector<double>& distCoeffs1,
                            cv::Mat& cameraMatrix2,  std::vector<double>& distCoeffs2);
    dualExtrinsicCalibration(cv::Size_<int> boardSize,float squareSize, cv::Mat& cameraMatrix1,cv::Mat& cameraMatrix2,  
                            const std::vector<double>& distCoeffs1, const std::vector<double>& distCoeffs2, int num_samples);

    ~dualExtrinsicCalibration();
    //bool copmuteTransformation(cv::Mat &img1, cv::Mat &img2);
    bool copmuteTransformation(const cv::Mat &img1,const cv::Mat &img2);
    bool continuousTransformation(const cv::Mat &img1,const cv::Mat &img2,std::vector<double> &orientation, std::vector<double> &position);
    bool continuousNAverageTransformation(const cv::Mat &img1,const cv::Mat &img2,std::vector<double> &orientation, std::vector<double> &position);
    bool averageTransformation();
    bool affine3ToMat(cv::Matx33f &A, cv::Mat &M  );
    Eigen::Vector4d avg_quaternion_markley();
    cv::Vec3d computeAverageTranslation();
    bool vector4dToCvQuat(const Eigen::Vector4d &q, cv::Quat<double> &Q);
    double getQX();
    double getQY();
    double getQZ();
    double getQW();
    double getTX();
    double getTY();
    double getTZ();

};



#endif