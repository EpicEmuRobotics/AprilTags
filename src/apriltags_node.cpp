/**
 *
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"
#include "AprilTagReader.h"

#define NUM_APRIL_TAGS 10

ros::Time last_lw_time, last_rw_time;

bool hasNewSpeeds = true;

int main(int argc, char** argv){
  ros::init(argc, argv, "AprilTagsNode");

  //ROS nodehandle
  ros::NodeHandle n;

  //Transform broadcaster to send the newly found tags to the TF tree
  tf::TransformBroadcaster tags_broadcaster;

  //The tags publisher will publish the pose of the tag (relative to the camera) on the 'apriltags' topic
  ros::Publisher tags_pub = n.advertise<geometry_msgs::PoseStamped>("apriltags", 100);

  //AprilTagReader constantly receives messages on the specified topic, and processes the image, finding april tags
  AprilTagReader reader;

  ros::Rate r(10.0);

  tf::TransformListener listener;

  while(n.ok()){
    ros::spinOnce();               // check for incoming messages

    ROS_INFO("Reading April Tags...");
    reader.read();

    ROS_INFO("Running... #tags: %ld",reader.getTags().size());

    for (int i=0; i<reader.getTags().size(); i++)
    {
      AprilTags::TagDetection td = reader.getTags()[i];
      int id = reader.getTags()[i].id;

      double x,y,z,roll,pitch,yaw;
      reader.getTransformInfo(td, x,y,z,roll,pitch,yaw);

      //Pitch = z axis
      //Yaw = x axis
      //Roll = y axis

      //pitch += PI;
      //pitch = -pitch;

      //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(-yaw, -roll, -pitch);

      //We only care about the pitch (z axis)
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -pitch);

      //publish to the tf tree
      geometry_msgs::TransformStamped tag_transform;
      tag_transform.header.stamp = reader.getImageReadTime();
      tag_transform.header.frame_id = "/camera_rgb_frame";

      stringstream ss;
      ss << "/april_tag["<<id<<"]";

      tag_transform.child_frame_id = ss.str().c_str();

      tag_transform.transform.translation.x = x;
      tag_transform.transform.translation.y = y;
      tag_transform.transform.translation.z = z;
      tag_transform.transform.rotation = odom_quat;

      //send the transform
      tags_broadcaster.sendTransform(tag_transform);
    }

    ros::spinOnce();               // send output ASAP
    if (reader.getDraw())
      cvWaitKey(10);
    r.sleep();
  }
}
