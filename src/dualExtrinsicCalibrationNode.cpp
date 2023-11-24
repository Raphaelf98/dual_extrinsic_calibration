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


void callback(const ImageConstPtr& image1, const ImageConstPtr& image2, 
              const sensor_msgs::CameraInfoConstPtr& camera_info1, const sensor_msgs::CameraInfoConstPtr& camera_info2)
{
 
  try
  {
    cv::imshow("OPENNI CAM", cv_bridge::toCvShare(image1, "bgr8")->image);
    cv::imshow("WEB CAM", cv_bridge::toCvShare(image2, "bgr8")->image);
    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image1->encoding.c_str());
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image2->encoding.c_str());
  }
  int k = cv::waitKey(1);
  
  if (k == 32)
  {
    
    boost::array<double, 9> K1 = camera_info1->K;
    boost::array<double, 9> K2 = camera_info2->K;
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
    cv::Size_<int> size(8,5);
    dualExtrinsicCalibration dualcalibrator(size, 0.03, K1_mat, K2_mat,camera_info1->D, camera_info2->D,10);
    dualcalibrator.copmuteTransformation(cv_bridge::toCvShare(image1, "bgr8")->image,cv_bridge::toCvShare(image2, "bgr8")->image);

    std::cout << "compute transformation .. "<<std::endl;
  }
  
  
}



class DualCalibrator
{
    public:
        DualCalibrator():
            nh{},samples_(0),
            sync_( MySyncPolicy(100),  image1_sub,  image2_sub),continuous_(0),
            timer(nh.createTimer(ros::Duration(0.1), &DualCalibrator::main_loop, this) )
         {
          cv::namedWindow("OPENNI CAM");
          cv::namedWindow("WEB CAM");
          
          nh.getParam("camera_1_image",img_topic_1_ );
          nh.getParam("camera_2_image",img_topic_2_ );
          nh.getParam("camera_1_info",inf_topic_1_ );
          nh.getParam("camera_2_info",inf_topic_2_ );

          
          image1_sub.subscribe(nh, img_topic_1_, 1);
          image2_sub.subscribe(nh, img_topic_2_, 1);
         

          sync_.registerCallback(boost::bind(&DualCalibrator::callback, this, _1, _2));

          sensor_msgs::CameraInfoConstPtr camInfomsg_1,camInfomsg_2;
          camInfomsg_1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(inf_topic_1_,nh);
          camInfomsg_2 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(inf_topic_2_,nh);

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
             cv::Size_<int> size(8,5);
             dualcalibrator_ = dualExtrinsicCalibration(size, 0.03, K1_mat, K2_mat,camInfomsg_1->D, camInfomsg_2->D,10);
               
          }
          pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("camera_pose_rel", 1);
         }

         void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)
         {
         
           
           try
           {
             cv::imshow("OPENNI CAM", cv_bridge::toCvShare(image1, "bgr8")->image);
             cv::imshow("WEB CAM", cv_bridge::toCvShare(image2, "bgr8")->image);
             
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
            if(dualcalibrator_.continuousNAverageTransformation(cv_bridge::toCvShare(image1, "bgr8")->image,cv_bridge::toCvShare(image2, "bgr8")->image,q,t))
            {
              ROS_INFO("Publishing Pose");
              // construct a pose message

              pose_stamped_.header.frame_id = "camera_frame";
              
              //NOTE: QUATERNIONS HAVE NEGATIVE SIGN
              pose_stamped_.pose.orientation.x = q[0];
              pose_stamped_.pose.orientation.y = q[1];
              pose_stamped_.pose.orientation.z = q[2];
              pose_stamped_.pose.orientation.w = q[3];

              pose_stamped_.pose.position.x = t[0];
              pose_stamped_.pose.position.y = t[1];
              pose_stamped_.pose.position.z = t[2];
            }
           
            
           }
           if (samples_ < 10 & !continuous_)
           {
            //press "space bar" to run in single sample mode
           if (k == 32)
           {
            
              dualcalibrator_.copmuteTransformation(cv_bridge::toCvShare(image1, "bgr8")->image,cv_bridge::toCvShare(image2, "bgr8")->image);
             std::cout << "compute transformation .. "<<std::endl;
             samples_ ++;
           }
           }
           else if(samples_==10)
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
              samples_ ++;
           }
           else
           {
              pose_stamped_.header.stamp = ros::Time::now();
              pose_pub_.publish(pose_stamped_); 
           }
  
            
      
           
           
           
         }
         void main_loop(const ros::TimerEvent &) const
         {
            // implement the state machine here
          
         }
         ~DualCalibrator()
         {
          
         }

    private:
        std::string img_topic_1_, img_topic_2_, inf_topic_1_, inf_topic_2_;
        typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
        message_filters::Subscriber<Image> image1_sub;
        message_filters::Subscriber<Image> image2_sub;
        message_filters::Subscriber<CameraInfo> camera1_sub;
        message_filters::Subscriber<CameraInfo> camera2_sub;
        Synchronizer<MySyncPolicy> sync_;
        dualExtrinsicCalibration dualcalibrator_;
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pose_pub_;
        geometry_msgs::PoseStamped pose_stamped_;
        ros::Timer timer;
        bool continuous_;
        int samples_;
};




int main(int argc, char * argv[])
{
    ros::init(argc, argv, "dual_extrinsic_calibration_node");
    DualCalibrator node;
    ros::spin();
    return 0;
}