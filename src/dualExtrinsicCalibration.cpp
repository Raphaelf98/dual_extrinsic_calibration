#include"dualExtrinsicCalibration.h"

/*dualExtrinsicCalibration::dualExtrinsicCalibration(cv::Size_<int> boardSize,float squareSize,
                            cv::Mat& cameraMatrix1, std::vector<double>& distCoeffs1,
                            cv::Mat& cameraMatrix2,  std::vector<double>& distCoeffs2)
{  

}*/
dualExtrinsicCalibration::dualExtrinsicCalibration(){}
void dualExtrinsicCalibration::getVecFromMat(std::vector<double> &vec, cv::Mat &M){
    for(int i = 0; i < M.rows; i++)
    {
    const double* Mi = M.ptr<double>(i);
    for(int j = 0; j < M.cols; j++)
        vec.push_back(Mi[j]);
    }
}
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
dualExtrinsicCalibration::dualExtrinsicCalibration(cv::Size_<int> boardSize,float squareSize, cv::Mat& cameraMatrix1,cv::Mat& cameraMatrix2,  
                            const std::vector<double>& distCoeffs1, const std::vector<double>& distCoeffs2)
{
    num_samples_ = 0;
    matxd_ = Eigen::MatrixXd(10,4);
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

bool dualExtrinsicCalibration::averageTransformation()
{   

   Eigen::Vector4d avg_quat = avg_quaternion_markley();
   cv::Quat<double> q;
   cv::Matx33d avgRot;
   vector4dToCvQuat( avg_quat,q);
   avgRot = q.toRotMat3x3();
   cv::Vec3d avg_trans = computeAverageTranslation();
    cv::Affine3d final_(avgRot,avg_trans);
    std::cout<< "Average Rotation Matrix: "<<final_.rotation()<<std::endl;
    std::cout<< "Average Translation Matrix: "<<final_.translation()<<std::endl;
   return 0;
}
cv::Vec3d dualExtrinsicCalibration::computeAverageTranslation()
{   
    return trans_acc_*(1.0/static_cast<double>(num_samples_));
}
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

    affine3ToMat(Q,input);
    cv::Quat<double> q = cv::Quat<double>::createFromRotMat(input);
    cvQuatToVector4d(q, vec4d);
    
    push_back(matxd_ ,vec4d, num_samples_);
    cv::Vec3d trans= H_1_2.translation();
    trans_acc_    = trans_acc_+trans;
    std::cout << "TRANS MATRIX: "<< trans<<std::endl;
    std::cout << "MATRIX: "<< trans_acc_<<std::endl;
    std::cout<<"H12 rotation: "<< H_1_2.rotation()<<std::endl;
    std::cout<<"Quaternion: "<< q<<std::endl;
    std::cout<<"H12 translation: "<< H_1_2.translation()<<std::endl;
    num_samples_++;
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