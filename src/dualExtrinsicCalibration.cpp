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
    return avg_quat_(1);
}
double dualExtrinsicCalibration::getQY()
{
    return avg_quat_(2);
}
double dualExtrinsicCalibration::getQZ()
{
    return avg_quat_(3);
}
double dualExtrinsicCalibration::getQW()
{
    return avg_quat_(0);
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
   cv::Quat<double> q;
   cv::Matx33d avgRot;
   vector4dToCvQuat( avg_quat_,q);
   avgRot = q.toRotMat3x3();
   avg_trans_ = computeAverageTranslation();
    cv::Affine3d final_(avgRot,avg_trans_);
    std::cout<< "Average Rotation Matrix: "<<final_.rotation()<<std::endl;
    std::cout<< "Average Translation Matrix: "<<final_.translation()<<std::endl;
   return 0;
}

bool dualExtrinsicCalibration::continuousTransformation(const cv::Mat &img1,const cv::Mat &img2,std::vector<double> &orientation, std::vector<double> &position)
{
    cv::Affine3<double> H1,H2,H2_inv,H_1_2;
    pose1_.estimatePose(img1);
    H1 = pose1_.getTransform();
   
    pose2_.estimatePose(img2);
    H2 = pose2_.getTransform();

    H2_inv = H2.inv();
    H_1_2 = H2_inv.concatenate(H1);
    cv::Matx33f Q = H_1_2.rotation();
    cv::Mat input;
    Eigen::Vector4d vec4d;

    affine3ToMat(Q,input);
    cv::Quat<double> q = cv::Quat<double>::createFromRotMat(input);
    cvQuatToVector4d(q, vec4d);
    
    orientation[0] = vec4d(0);
    orientation[1] = vec4d(1);
    orientation[2] = vec4d(2);
    orientation[3] = vec4d(3);
    
    cv::Vec3d trans= H_1_2.translation();
    position[0] = trans(0);
    position[1] = trans(1);
    position[2] = trans(2);
    std::cout << "TRANS MATRIX: "<< trans<<std::endl;
    std::cout << "MATRIX: "<< trans_acc_<<std::endl;
    std::cout<<"H12 rotation: "<< H_1_2.rotation()<<std::endl;
    std::cout<<"Quaternion: "<< q<<std::endl;
    std::cout<<"H12 translation: "<< H_1_2.translation()<<std::endl;

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
        
        if(pose1_.estimatePose(img1) && pose2_.estimatePose(img2))
        {
            H1 = pose1_.getTransform();
            H2 = pose2_.getTransform();
            H2_inv = H2.inv();
            H_1_2 = H2_inv.concatenate(H1);
            cv::Matx33f Q = H_1_2.rotation();
            cv::Mat input;
            Eigen::Vector4d vec4d;

            affine3ToMat(Q,input);
            cv::Quat<double> q = cv::Quat<double>::createFromRotMat(input);
            cvQuatToVector4d(q, vec4d);

            push_back(matxd_ ,vec4d, sample_);
            cv::Vec3d trans= H_1_2.translation();
            trans_acc_    = trans_acc_+trans;
       
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
        orientation[0] = avg_quat_(1);
        orientation[1] = avg_quat_(2);
        orientation[2] = avg_quat_(3);
        orientation[3] = avg_quat_(0);
        
        cv::Quat<double> q;
        vector4dToCvQuat(avg_quat_,q);
        cv::Vec3d angles = q.toEulerAngles(cv::QuatEnum::INT_ZYX);
        std::cout<<"QUATERNIONS: "<<q[0]<<","<<q[1]<<","<<q[2]<<","<<q[3]<<"    ANGLES: "<<(180/M_PI)*angles <<std::endl;
        matxd_ = Eigen::MatrixXd(10,4);
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
    cv::Affine3<double> H1,H2,H2_inv,H_1_2;
    pose1_.estimatePose(img1);
    H1 = pose1_.getTransform();
   
    pose2_.estimatePose(img2);
    H2 = pose2_.getTransform();

    H2_inv = H2.inv();
    H_1_2 = H2_inv.concatenate(H1);
    cv::Matx33f Q = H_1_2.rotation();
    cv::Mat input;
    Eigen::Vector4d vec4d;

    push_back(matxd_ ,vec4d, sample_);
    cv::Vec3d trans= H_1_2.translation();
    trans_acc_    = trans_acc_+trans;
    std::cout << "TRANS MATRIX: "<< trans<<std::endl;
    std::cout << "MATRIX: "<< trans_acc_<<std::endl;
    std::cout<<"H12 rotation: "<< H_1_2.rotation()<<std::endl;
    
    std::cout<<"H12 translation: "<< H_1_2.translation()<<std::endl;
    sample_++;
}
bool dualExtrinsicCalibration::affine3ToMat(cv::Matx33f &A, cv::Mat &M )
{
    M = cv::Mat::zeros(3,3, CV_64FC1);

    for(size_t i= 0; i<3; i++){
        for(size_t j= 0; j<3; j++){
            M.at<double>(i,j) = A(i,j);

        }
    }
    return 1;
}

bool dualExtrinsicCalibration::cvQuatToVector4d(const cv::Quat<double> &Q, Eigen::Vector4d &q)
{
    for(size_t i= 0; i<4; i++)
    {
        q(i) = Q[i];
    }
    return 1;
}
bool dualExtrinsicCalibration::vector4dToCvQuat(const Eigen::Vector4d &q, cv::Quat<double> &Q)
{
    for(size_t i= 0; i<4; i++)
    {
        Q[i] = q(i);
    }
    return 1;
}