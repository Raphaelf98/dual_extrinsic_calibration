#include"dualExtrinsicCalibration.h"

dualExtrinsicCalibration::dualExtrinsicCalibration(){}
/*
Convert 2D Mat to std::vector<double>.
*/
void dualExtrinsicCalibration::getVecFromMat(std::vector<double> &vec, cv::Mat &M){
    for(int i = 0; i < M.rows; i++)
    {
    const double* Mi = M.ptr<double>(i);
    for(int j = 0; j < M.cols; j++)
        vec.push_back(Mi[j]);
    }
}
/*
Constructor to read camera parameters from yaml file
*/
dualExtrinsicCalibration::dualExtrinsicCalibration(cv::Size_<int> boardSize,float squareSize, std::string calib_yaml_1, std::string calib_yaml_2)
{
    
    cv::FileStorage fs_1(calib_yaml_1, cv::FileStorage::READ);
    cv::FileStorage fs_2(calib_yaml_2, cv::FileStorage::READ);
    cv::Mat cameraMatrix1;
    cv::Mat distortionCoefficient1;
    cv::Mat cameraMatrix2;
    cv::Mat distortionCoefficient2;
    fs_1["camera_matrix"] >> cameraMatrix1;
    fs_1["distortion_coefficients"] >> distortionCoefficient1;
    fs_2["camera_matrix"] >> cameraMatrix2;
    fs_2["distortion_coefficients"] >> distortionCoefficient2;
    
    std::vector<double> dC1;
    std::vector<double> dC2;
    std::cout<< "cameraMatrix1: "<<cameraMatrix1  <<std::endl;
    std::cout<< "cameraMatrix2: "<<cameraMatrix2  <<std::endl;
    
   
    getVecFromMat(dC1,distortionCoefficient1);
    getVecFromMat(dC2,distortionCoefficient2);
     std::cout<< "distortionCoefficient1" <<std::endl;
    for (auto &i : dC1){
        std::cout<< i <<std::endl;
    }
     std::cout<< "distortionCoefficient2" <<std::endl;
    for (auto &i : dC2){
        std::cout<< i <<std::endl;
    }

    pose1_ = poseEstimation();
    pose2_ = poseEstimation();
    pose1_.initialize(boardSize, squareSize, cameraMatrix1, dC1);
    pose2_.initialize(boardSize, squareSize ,cameraMatrix2, dC2);

}   
/*
Constructor to initialize class object with camera matrices and distortion parameters directly. 
The following Paramters are required: 
    1. boardSize - Size of chessboard
    2. squareSize - Distance between two chessboard corners in m
    3. cameraMatrix1, cameraMatrix2 - Camera matrices in cv::Mat format.
    4. distCoeffs1, distCoeffs2 - Distortion paramters based on opencv camera model.
    5. num_samples - Number of samples to collect before cumputing mean rotation and translation.
*/
dualExtrinsicCalibration::dualExtrinsicCalibration(cv::Size_<int> boardSize,float squareSize, cv::Mat& cameraMatrix1,cv::Mat& cameraMatrix2,  
                            const std::vector<double>& distCoeffs1, const std::vector<double>& distCoeffs2, int num_samples)
{
    counter_=0;
    sample_ = 0;
    num_samples_=num_samples;
    matxd_ = Eigen::MatrixXd(num_samples_,4);
    pose1_ = poseEstimation();
    pose2_ = poseEstimation();
    std::cout<<"Chessboard Size: "<<boardSize.width << "x"<<boardSize.height << ", size:" << squareSize<<", samples: "<<num_samples_ <<std::endl;
    pose1_.initialize(boardSize, squareSize, cameraMatrix1, distCoeffs1);
    pose2_.initialize(boardSize, squareSize ,cameraMatrix2, distCoeffs2);

}
dualExtrinsicCalibration::~dualExtrinsicCalibration()
{
    
}
/*Quaternion Averaging
Since quaternions are not regular vectors, but rather representations of orientation, an average quaternion cannot just be obtained by taking a weighted mean. 
This function implements the work done by F. Landis Merkley to calculate the average quaternion. The algorithm is explained by F. Landis Markley. For this particular implementation, 
I would also like to reference Mandar Harshe.
While being basic and straightforward, this algorithm is compared with rotqrmean from VoiceBox and found to produce quite similar results, 
yet it is more elegant, much simpler to implement and follow. (Though, there might be difference in signs)
Source: http://tbirdal.blogspot.com/
*/
Eigen::Vector4d dualExtrinsicCalibration::avg_quaternion_markley(){

    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
    int M = matxd_.rows();

    for(int i=0; i<M; i++){
        Eigen::Vector4d q = matxd_.row(i);
    if (q[0]<0)
     q = -q;
        A = q*q.adjoint() + A;
    }

    A=(1.0/M)*A;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(A);
    Eigen::Vector4d qavg=eig.eigenvectors().col(3);
    return qavg;
}
double dualExtrinsicCalibration::getQX()
{
    return avg_quat_(0);
}
double dualExtrinsicCalibration::getQY()
{
    return avg_quat_(1);
}
double dualExtrinsicCalibration::getQZ()
{
    return avg_quat_(2);
}
double dualExtrinsicCalibration::getQW()
{
    return avg_quat_(3);
}
double dualExtrinsicCalibration::getTX()
{
    return avg_trans_(0);
}
double dualExtrinsicCalibration::getTY()
{
    return avg_trans_(1);
}
double dualExtrinsicCalibration::getTZ()
{
    return avg_trans_(2);
}
/*
Computes average rotation matrix and average rotation vector.
*/
bool dualExtrinsicCalibration::averageTransformation()
{   

   avg_quat_ = avg_quaternion_markley();

   avg_trans_ = computeAverageTranslation();     
  
   return 0;
}

/*
Computes n (num_samples_) rotation matrices and translation vectors from image 1 and image 2. 
The relative transform between camera 1 and camera 2 is computed by solving H_1_2 = H_2^(-1)H_1.
If num_samples_ of relative transforms from both img1 and img2 were successfully computed, their average is calcualted.
Results are given as position (x,y,z) and orientation(x,y,z,w) vectors
*/
bool dualExtrinsicCalibration::continuousNAverageTransformation(const cv::Mat &img1,const cv::Mat &img2,std::vector<double> &orientation, std::vector<double> &position)
{
    if(counter_ <= num_samples_)
    {
         cv::Affine3<double> H1,H2,H2_inv,H_1_2;
        if(pose1_.estimatePose(img1) & pose2_.estimatePose(img2))
        {
            H1 = pose1_.getTransform();
            H2 = pose2_.getTransform();
            H2_inv = H2.inv();
            H_1_2 = H2_inv.concatenate(H1);
            cv::Matx33f Q = H_1_2.rotation();
            Eigen::Matrix3f eigRotMat;
            affine3ToEigen(Q,eigRotMat);
            Eigen::Quaternion<float> eigQuat(eigRotMat);
            Eigen::Vector4d vec4d;
            vec4d(0) = eigQuat.x();
            vec4d(1) = eigQuat.y();
            vec4d(2) = eigQuat.z();
            vec4d(3) = eigQuat.w();
            push_back(matxd_ ,vec4d, sample_);
            cv::Vec3d trans= H_1_2.translation();
            trans_acc_    = trans_acc_ + trans;
            sample_++;
            counter_++;
        }
        return false;
    }
    else{
        cv::Vec3d avg_trans = trans_acc_*(1.0/static_cast<double>(sample_));
        position[0] = avg_trans(0);
        position[1] = avg_trans(1);
        position[2] = avg_trans(2);
        trans_acc_(0)= 0;
        trans_acc_(1)= 0;
        trans_acc_(2)= 0;
        avg_quat_ = avg_quaternion_markley();
        orientation[0] = avg_quat_(0);
        orientation[1] = avg_quat_(1);
        orientation[2] = avg_quat_(2);
        orientation[3] = avg_quat_(3);
        
        matxd_ = Eigen::MatrixXd(num_samples_,4);
        counter_=0;
        sample_=0;
        return true;
    }
}
/*
Computes average translation from trans_acc_ and number of samples accumulated 
*/
cv::Vec3d dualExtrinsicCalibration::computeAverageTranslation()
{   
    return trans_acc_*(1.0/static_cast<double>(sample_));
}
/*
Computes rotation matrices and translation vectors from image 1 and image 2. 
The relative transform between camera 1 and camera 2 is computed by solving H_1_2 = H_2^(-1)H_1.
Orientation Quaternions are pushed back to matxd_ (Eigen::MatrixXd) and trans_acc_ sums up translation vectors. 
Used in combination with averageTransformation class member function. 
*/
bool dualExtrinsicCalibration::copmuteTransformation(const cv::Mat &img1,const cv::Mat &img2)
{
    if(pose1_.estimatePose(img1) & pose2_.estimatePose(img2))
    {
        cv::Affine3<double> H1,H2,H2_inv,H_1_2;

        H1 = pose1_.getTransform();
        H2 = pose2_.getTransform();

        H2_inv = H2.inv();
        H_1_2 = H2_inv.concatenate(H1);
        cv::Matx33f Q = H_1_2.rotation();

        Eigen::Matrix3f eigRotMat;
        affine3ToEigen(Q,eigRotMat);
        Eigen::Quaternion<float> eigQuat(eigRotMat);
        Eigen::Vector4d vec4d;
        vec4d(0) = eigQuat.x();
        vec4d(1) = eigQuat.y();
        vec4d(2) = eigQuat.z();
        vec4d(3) = eigQuat.w();

        push_back(matxd_ ,vec4d, sample_);
        cv::Vec3d trans= H_1_2.translation();
        trans_acc_    = trans_acc_+trans;
        std::cout << "TRANS MATRIX: "<< trans<<std::endl;
        std::cout << "MATRIX: "<< trans_acc_<<std::endl;
        std::cout<<"H12 rotation: "<< H_1_2.rotation()<<std::endl;
        std::cout<<"H12 translation: "<< H_1_2.translation()<<std::endl;
        sample_++;
        return true;
    }
    else{
        return false;
    }
}
void dualExtrinsicCalibration::affine3ToMat(const cv::Matx33f &A, cv::Mat &M )
{
    M = cv::Mat::zeros(3,3, CV_64FC1);

    for(size_t i= 0; i<3; i++){
        for(size_t j= 0; j<3; j++){
            M.at<double>(i,j) = A(i,j);

        }
    }
   
}
void dualExtrinsicCalibration::affine3ToEigen(const cv::Matx33f &A, Eigen::Matrix3f &M ){
    
    for(size_t i= 0; i<3; i++)
    {
        for(size_t j=0; j<3; j++)
        {
            M(i,j) = A(i,j);
        }
    }
}
