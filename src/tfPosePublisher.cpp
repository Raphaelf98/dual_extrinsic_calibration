#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

std::string turtle_name;



void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  /*transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));*/
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_pose_broadcaster");
  
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("calibrated_camera/rel_pose", 10, &poseCallback);

  ros::spin();
  return 0;
};