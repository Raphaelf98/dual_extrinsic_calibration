#include "dualExtrinsicCalibration.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <geometry_msgs/PoseStamped.h>
using namespace sensor_msgs;
using namespace message_filters;


/* DualCalibrator class 
This class serves as a wrapper for a ros node to compute the relative camera pose between two cameras. 
It subscribes to two camera info and two camera image topics and 
after successfully computing a relative pose a stamped pose is publised.
A Message filter ensures approximately synchronous images.  
*/
class DualCalibrator
{
    public:
        /*
        Initialize the node with the number of samples that
        should be used for computing the average rotation and translation (see dualExtrinsicCalibration.cpp)
        */
        DualCalibrator():
            nh_{},sample_(0),
            sync_(MySyncPolicy_(100),image1_sub_ ,image2_sub_),continuous_(0)
            
         {
          cv::namedWindow("Camera 1");
          cv::namedWindow("Camera 2");
          //retrieve parameters from launch file
          nh_.getParam("camera_1_image",img_topic_1_ );
          nh_.getParam("camera_2_image",img_topic_2_ );

          nh_.getParam("camera_1_info",inf_topic_1_ );
          nh_.getParam("camera_2_info",inf_topic_2_ );
          
          double a,b;
          nh_.getParam("chessboard_size_a", a);
          nh_.getParam("chessboard_size_b", b);
         
          size_.width  = (int) a;
          size_.height  = (int) b;
          nh_.getParam("chessboard_length",c_length_ );
          nh_.getParam("sample_size", num_samples_);
          
          image1_sub_.subscribe(nh_, img_topic_1_, 1);
          image2_sub_.subscribe(nh_, img_topic_2_, 1);
         
          
          sync_.registerCallback(boost::bind(&DualCalibrator::callback, this, _1, _2));

          sensor_msgs::CameraInfoConstPtr camInfomsg_1, camInfomsg_2;
          camInfomsg_1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(inf_topic_1_,nh_);
          camInfomsg_2 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(inf_topic_2_,nh_);
          //Get camera matrices and distortion vectors from camera info topics. Instantiate dualExtrinsicCalibration object. 
          if(camInfomsg_1 != NULL && camInfomsg_2 != NULL)
          {
             boost::array<double, 9> K1 = camInfomsg_1->K;
             boost::array<double, 9> K2 = camInfomsg_2->K;
             cv::Mat K1_mat = cv::Mat(3, 3, CV_64F);
             cv::Mat K2_mat = cv::Mat(3, 3, CV_64F);
             int col = 0;
             int row = 0;
             for(size_t i = 0; i<K1.size(); i++)
             {
               
               K1_mat.at<double>(row,col) = K1[i];
               K2_mat.at<double>(row,col) = K2[i];
               col++;
               if ((i+1)%3==0)
               {
                 col = 0;
                 row ++;
               }
            
             }
            
            
           dualcalibrator_ = dualExtrinsicCalibration(size_, c_length_, K1_mat, K2_mat,camInfomsg_1->D, camInfomsg_2->D,num_samples_);
               
          }
          
          pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("camera_pose_rel", 1);
         }
        /*
        Image subscriber callback. Reads from image messages and passes them to dualExtrinsicCalibration class member functions. 
        Node can be run in two modes:
        1. Continous mode
          For this Press "c". num_samples number of images are collected before average transform is computed and published continuously.
        2. Single mode
          For this press "space bar". It collects images from both cameras whenever the space bar is pressed. 
          num_samples determins after how many images an average transform is computed

        */
        void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)
         {
         
           
           try
           {
             cv::imshow("Camera 1", cv_bridge::toCvShare(image1, "bgr8")->image);
             cv::imshow("Camera 2", cv_bridge::toCvShare(image2, "bgr8")->image);
             
           }
           catch (cv_bridge::Exception& e)
           {
             ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image1->encoding.c_str());
             ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image2->encoding.c_str());
           }
           int k = cv::waitKey(1);
           //Press "c" to run in continuous mode
           if (k == 99)
           {
            continuous_ = true;
           }

           if(continuous_)
           {
            std::vector<double> q(4);
            std::vector<double> t(3);
            if(dualcalibrator_.continuousNAverageTransformation(cv_bridge::toCvShare(image1, "bgr8")->image, cv_bridge::toCvShare(image2, "bgr8")->image, q, t))
            {
              ROS_INFO("Publishing Pose");
              // construct a pose message

              pose_stamped_.header.frame_id = "camera_frame";
              
              
              pose_stamped_.pose.orientation.x = q[1];
              pose_stamped_.pose.orientation.y = q[2];
              pose_stamped_.pose.orientation.z = q[3];
              pose_stamped_.pose.orientation.w = q[0];

              pose_stamped_.pose.position.x = t[0];
              pose_stamped_.pose.position.y = t[1];
              pose_stamped_.pose.position.z = t[2];
            }
           
            
           }
           if (sample_ < num_samples_ & !continuous_)
           {
            
            //press "space bar" to run in single sample mode
            if (k == 32)
            {

              if(dualcalibrator_.copmuteTransformation(cv_bridge::toCvShare(image1, "bgr8")->image,cv_bridge::toCvShare(image2, "bgr8")->image)){
                std::cout << "Compute transformation .. "<<std::endl;
                sample_ ++;
              }
              else{
                std::cout << "Failed to compute transformation .. "<<std::endl;
              }
            }
           }
           else if(sample_==num_samples_)
           {
              dualcalibrator_.averageTransformation();
              cv::destroyAllWindows();
              ROS_INFO("Publishing Pose");
              // construct a pose message

              pose_stamped_.header.frame_id = "camera_frame";
              
              pose_stamped_.pose.orientation.x = dualcalibrator_.getQX();
              pose_stamped_.pose.orientation.y = dualcalibrator_.getQY();
              pose_stamped_.pose.orientation.z = dualcalibrator_.getQZ();
              pose_stamped_.pose.orientation.w = dualcalibrator_.getQW();

              pose_stamped_.pose.position.x = dualcalibrator_.getTX();
              pose_stamped_.pose.position.y = dualcalibrator_.getTY();
              pose_stamped_.pose.position.z = dualcalibrator_.getTZ();
              sample_ ++;
           }
           else
           {
              pose_stamped_.header.stamp = ros::Time::now();
              pose_pub_.publish(pose_stamped_); 
           }
  
            
      
           
           
           
         }
        
         ~DualCalibrator()
         {
          
         }

    private:
        std::string img_topic_1_, img_topic_2_, inf_topic_1_, inf_topic_2_;
        typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy_;
        message_filters::Subscriber<Image> image1_sub_;
        message_filters::Subscriber<Image> image2_sub_;
        message_filters::Subscriber<CameraInfo> camera1_sub_;
        message_filters::Subscriber<CameraInfo> camera2_sub_;
        Synchronizer<MySyncPolicy_> sync_;
        dualExtrinsicCalibration dualcalibrator_;
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pose_pub_;
        geometry_msgs::PoseStamped pose_stamped_;
        ros::Timer timer_;
        bool continuous_;
        int sample_, num_samples_;
        cv::Size_<int> size_;
        double c_length_;
        
};




int main(int argc, char * argv[])
{
    ros::init(argc, argv, "dual_extrinsic_calibration_node");
    DualCalibrator node;
    ros::spin();
    return 0;
}