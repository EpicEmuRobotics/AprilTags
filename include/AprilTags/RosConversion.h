/////////////////////////////////////////////////////////////////////
/// rosConversion: class that provides a lot of conversions
/// between the base apriltagreader and ros transforms/poses/other
/// /////////////////////////////////////////////////////////////////


#include <ros/ros.h>
#include "AprilTagReader.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#ifndef ROS_CONVERSION_H
#define ROS_CONVERSION_H

class RosConversion {
public:

  /**
   * this function takes a tagdetection and gets the x,y,z, roll, pitch, yaw
   * (in ros terminology) and sets them to the references passed in.
   */
  void GetTransformInfo(AprilTags::TagDetection& detection,
                                     double& x, double& y,double& z,
                                     double& roll, double& pitch,double& yaw) const;

  /**
   * getTranform returns a ros::tf:Transform from the AprilTags::TagDetection
   * @param  detection AprilTag in the camera frame
   * @param  reader    Reader that has the camera params stored
   *
   * @return           ros tf transform
   */
  tf::Pose GetPoseFromTagDetection(AprilTags::TagDetection& detection, AprilTagReader& reader) const;


}; // tfConversion

#endif