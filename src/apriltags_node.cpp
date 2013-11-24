/**
 * 
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float32.h"
#include "AprilTagReader.h"

#define NUM_APRIL_TAGS 10

ros::Time last_lw_time, last_rw_time;

bool hasNewSpeeds = true;

int main(int argc, char** argv){
  ros::init(argc, argv, "AprilTagsNode");

  ros::NodeHandle n;

  std::vector<tf::TransformBroadcaster> tags_broadcaster(10);

  AprilTagReader reader;
  reader.setup();

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);

  while(n.ok()){
    ros::spinOnce();               // check for incoming messages

    ROS_INFO("Reading April Tags...");
    reader.read();

    current_time = ros::Time::now();

    ROS_INFO("Running... #tags: %ld, time: %lf",reader.getTags().size(), current_time.toSec());

    for (int i=0; i<reader.getTags().size(); i++)
    {
      AprilTags::TagDetection td = reader.getTags()[i];
      int id = reader.getTags()[i].id;

      double x,y,z,roll,pitch,yaw;
      reader.getTransformInfo(td, x,y,z,roll,pitch,yaw);

      //Pitch = z axis
      //Yaw = x axis
      //Roll = y axis

      pitch += PI;
      pitch = -pitch;

      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(yaw, roll, pitch);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "camera_link";

      stringstream ss;
      ss << "april_tag["<<id<<"]";
      
      odom_trans.child_frame_id = ss.str().c_str();

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = z;
      odom_trans.transform.rotation = odom_quat;

      //send the transform
      tags_broadcaster[id].sendTransform(odom_trans);
    }

    cvWaitKey(10);
    last_time = current_time;
    r.sleep();
  }
}
