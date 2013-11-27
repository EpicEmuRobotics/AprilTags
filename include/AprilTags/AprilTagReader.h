// April tags detector and various families that can be selected by command line option
#include "TagDetector.h"
#include "TagDetection.h"
#include "Tag16h5.h"
#include "Tag25h7.h"
#include "Tag25h9.h"
#include "Tag36h9.h"
#include "Tag36h11.h"

// Needed for getopt / command line options processing
#include <unistd.h>

//ROS specific stuff
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


#ifndef APRIL_TAG_READER_H
#define APRIL_TAG_READER_H


#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;


class AprilTagReader {
private:
  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;

  bool m_draw; // draw image and April tag detections?
  bool m_arduino; // send tag detections to serial port?

  int m_width; // image size in pixels
  int m_height;
public:
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;
private:
  int m_deviceId; // camera id (in case of multiple cameras)
  cv::VideoCapture m_cap;

  int m_exposure;
  int m_gain;
  int m_brightness;

  bool hasNewImage;

  cv::Mat m_image;
  cv::Mat m_image_gray;

  std::string window_name;

  void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);

  ///ROS STUFF
  
  ros::NodeHandle nh;
  image_transport::ImageTransport m_it;
  image_transport::Subscriber ros_image_sub;

public:

  // default constructor
  AprilTagReader() :
    // default settings, most can be modified through command line options (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    m_draw(true),
    m_arduino(false),

    m_width(640),
    m_height(480),
    m_tagSize(0.166),
    m_fx(600),
    m_fy(600),
    m_px(m_width/2),
    m_py(m_height/2),

    m_exposure(-1),
    m_gain(-1),
    m_brightness(-1),

    m_deviceId(0),

    hasNewImage(false),
    m_it(nh)
  {

    window_name = std::string("april_tags_output");

    // prepare window for drawing the camera images
    if (m_draw) {
      cv::namedWindow(window_name, 1);
    }

    // ROS topic to listen to that sends the images (currently supports rgb)
    ros_image_sub= m_it.subscribe("/camera/rgb/image_color", 1, &AprilTagReader::imageCallback, this);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& img);

  // changing the tag family
  void setTagCodes(string s);

  // parse command line options to change default behavior
  void parseOptions(int argc, char* argv[]);

  void setup();

  void print_detection(AprilTags::TagDetection& detection);

  // The processing loop where images are retrieved, tags detected,
  // and information about detections generated
  void read();

  std::vector<AprilTags::TagDetection> getTags() { return m_lastReadTags; };

  std::vector<AprilTags::TagDetection> m_lastReadTags;

  ros::Time m_lastImageTime;
  ros::Time GetLastImageTime(){return m_lastImageTime; };

  /**
   * Normalize angle to be within the interval [-pi,pi].
   */
  inline double standardRad(double t);

}; // AprilTagReader

#endif