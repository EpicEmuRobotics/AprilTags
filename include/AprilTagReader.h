// April tags detector and various families that can be selected by command line option
#include "TagDetector.h"
#include "TagDetection.h"
#include "Tag16h5.h"
#include "Tag25h7.h"
#include "Tag25h9.h"
#include "Tag36h9.h"
#include "Tag36h11.h"

//ROS specific stuff
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;


class AprilTagReader {

  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;

  bool m_draw; // draw image and April tag detections?

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  bool hasNewImage;

  cv::Mat m_image;
  cv::Mat m_image_gray;

  std::string window_name;

  void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);


  ///ROS STUFF

  ros::NodeHandle nh;
  image_transport::ImageTransport m_it;
  image_transport::Subscriber m_image_sub;

  std::string m_image_topic;

public:
  AprilTagReader();

  void imageCallback(const sensor_msgs::ImageConstPtr& img);

  // changing the tag family
  void setTagCodes(string s);

  void processParams(ros::NodeHandle);

  void setup();

  void print_detection(AprilTags::TagDetection& detection);

  // The processing loop where images are retrieved, tags detected,
  // and information about detections generated
  void read();

  std::vector<AprilTags::TagDetection> getTags() { return m_lastReadTags; };

  std::vector<AprilTags::TagDetection> m_lastReadTags;
  /**
   * Normalize angle to be within the interval [-pi,pi].
   */
  inline double standardRad(double t);

  void getTransformInfo(AprilTags::TagDetection& detection, double& x, double& y, double& z,
                       double& roll, double& pitch, double& yaw);
}; // AprilTagReader
