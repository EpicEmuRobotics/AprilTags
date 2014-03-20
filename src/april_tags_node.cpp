/**
 * April tag reader node. Publishes TF tree and messages
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <april_tags/AprilTag.h>
#include "AprilTagReader.h"


geometry_msgs::TransformStamped getTransformStamped(int id, ros::Time imageReadTime, double x, double y, double z, double roll, double pitch, double yaw)
{
  //Pitch = z axis
  //Yaw = x axis
  //Roll = y axis

  //pitch += PI;
  //pitch = -pitch;

  //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(-yaw, -roll, -pitch);

  //We only care about the pitch (z axis)
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -pitch);

  geometry_msgs::TransformStamped tag_transform;
  tag_transform.header.stamp = imageReadTime;
  tag_transform.header.frame_id = "camera_rgb_frame";

  stringstream ss;
  ss << "april_tag["<<id<<"]";

  tag_transform.child_frame_id = ss.str().c_str();

  tag_transform.transform.translation.x = x;
  tag_transform.transform.translation.y = y;
  tag_transform.transform.translation.z = z;
  tag_transform.transform.rotation = odom_quat;

  return tag_transform;
}

april_tags::AprilTag getNotificationMessage(int id, geometry_msgs::TransformStamped transform)
{
  //Use for display and planning purposes
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header = transform.header;
  poseStamped.pose.position.x = transform.transform.translation.x;
  poseStamped.pose.position.y = transform.transform.translation.y;
  poseStamped.pose.position.z = transform.transform.translation.z;
  poseStamped.pose.orientation = transform.transform.rotation;

  april_tags::AprilTag tag;
  tag.pose = poseStamped;
  tag.tag_id = id;

  return tag;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "april_tag_node");

  ROS_INFO("Starting April Tag Reader node");

  //ROS nodehandle
  ros::NodeHandle nh;

  //AprilTagReader constantly receives messages on the specified topic, and processes the image, finding april tags
  AprilTagReader reader(nh);

  // Publisher to send out the update message
  ros::Publisher tags_pub;
  //Transform broadcaster to send the newly found tags to the TF tree
  tf::TransformBroadcaster tags_broadcaster;

  ros::Rate r(10.0);

  tags_pub = nh.advertise<april_tags::AprilTag>("april_tags", 100);

  while(nh.ok()){
    //ROS_INFO("Reading April Tags...");
    reader.read();

    //ROS_INFO("Running... #tags: %ld",reader.getTags().size());

    for (int i=0; i<reader.getTags().size(); i++)
    {
      int id = reader.getTags()[i].id;
      AprilTags::TagDetection td = reader.getTags()[i];

      double x,y,z,roll,pitch,yaw;
      reader.getTransformInfo(td, x,y,z,roll,pitch,yaw);

      geometry_msgs::TransformStamped transformStamped = getTransformStamped(id, reader.getImageReadTime(), x,y,z,roll, pitch, yaw);
      //publish to the tf tree
      tags_broadcaster.sendTransform(transformStamped);

      april_tags::AprilTag at = getNotificationMessage(i, transformStamped);
      tags_pub.publish(at);
    }

    ros::spinOnce();               // send output ASAP
    if (reader.getDraw())
      cvWaitKey(10);
    r.sleep();
  }
}
