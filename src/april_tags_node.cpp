/**
 * April tag reader node. Publishes TF tree and messages
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <april_tags/AprilTag.h>
#include <april_tags/AprilTagList.h>
#include "AprilTagReader.h"

std::string tf_prefix;

geometry_msgs::TransformStamped getTransformStamped(int id, ros::Time imageReadTime, double x, double y, double z, double roll, double pitch, double yaw)
{
  //Pitch = z axis
  //Yaw = x axis
  //Roll = y axis

  //pitch += PI;
  //pitch = -pitch;

  // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(-yaw, -roll, -pitch);

  //We only care about the pitch (z axis)
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -pitch);

  geometry_msgs::TransformStamped tag_transform;
  tag_transform.header.stamp = imageReadTime;

  tag_transform.header.frame_id = tf_prefix + std::string("/camera_link");

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

  //Get the tf prefix
  if (nh.getParam("april_tags/tf_prefix", tf_prefix))
  {
    ROS_INFO_STREAM("Set tf_prefix to: "<<tf_prefix);
  }
  else
  {
    tf_prefix = std::string("");
  }

  // Publisher to send out the tag message
  ros::Publisher single_tag_pub;
  // Publisher to send out the update message
  ros::Publisher tags_pub;

  //Transform broadcaster to send the newly found tags to the TF tree
  tf::TransformBroadcaster tags_broadcaster;

  ros::Rate r(3.0);

  single_tag_pub = nh.advertise<april_tags::AprilTag>("april_tag", 100);
  tags_pub = nh.advertise<april_tags::AprilTagList>("april_tags", 100);

  while(nh.ok()){
    ros::spinOnce();
    //ROS_INFO("Reading April Tags...");
    reader.read();

    //ROS_INFO("Running... #tags: %ld",reader.getTags().size());

    //List of all april tags seen in the image
    april_tags::AprilTagList tags_list;

    for (int i=0; i<reader.getTags().size(); i++)
    {
      AprilTags::TagDetection td = reader.getTags()[i];
      int id = td.id;

      double x,y,z,roll,pitch,yaw;
      reader.getTransformInfo(td, x,y,z,roll,pitch,yaw);

      geometry_msgs::TransformStamped transformStamped = getTransformStamped(id, reader.getImageReadTime(), x,y,z,roll, pitch, yaw);
      //publish to the tf tree
      tags_broadcaster.sendTransform(transformStamped);

      april_tags::AprilTag at = getNotificationMessage(id, transformStamped);
      single_tag_pub.publish(at);

      //add to all tag broadcast
      tags_list.poses.push_back(at.pose);
      tags_list.tag_ids.push_back(at.tag_id);
    }

    tags_pub.publish(tags_list);

    //Publish a single tag with
    if (reader.getTags().size() == 0)
    {
      april_tags::AprilTag at;
      at.tag_id = -1;
      // single_tag_pub.publish(at);
    }

    if (reader.getDraw())
      cvWaitKey(10);
    r.sleep();
  }
}
