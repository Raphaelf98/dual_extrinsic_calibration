#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

std::string turtle_name;



void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
  
  
}
class TF_Broadcaster{
  public:
    
    TF_Broadcaster()
    {
      node_.getParam("parent_optical_frame", parent_optical_frame_);
      ROS_INFO("Got parent frame:%s ", parent_optical_frame_.c_str());
      sub_= node_.subscribe<geometry_msgs::PoseStamped>( "camera_pose_rel", 10, &TF_Broadcaster::poseCallback, this);
      
    }
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
      
      transform_.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
  
      tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w );
      static tf::TransformBroadcaster br_;
      transform_.setRotation(q);
      br_.sendTransform(tf::StampedTransform(transform_,msg->header.stamp , parent_optical_frame_, msg->header.frame_id));
    }
  private:
    ros::NodeHandle node_;
    tf::Transform transform_;
    
    std::string parent_optical_frame_;
    ros::Subscriber sub_;

};
//tf::TransformBroadcaster TF_Broadcaster::br_;
int main(int argc, char** argv){
  ros::init(argc, argv, "tf_pose_broadcaster");

  TF_Broadcaster tf_pub;
  ros::spin();
  return 0;
};

