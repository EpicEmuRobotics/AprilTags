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

#include <ros/ros.h>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;


class AprilTagReader {

  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;

  bool m_draw; // draw image and April tag detections?
  bool m_arduino; // send tag detections to serial port?

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  int m_deviceId; // camera id (in case of multiple cameras)
  cv::VideoCapture m_cap;

  int m_exposure;
  int m_gain;
  int m_brightness;

  cv::Mat m_image;
  cv::Mat m_image_gray;

  std::string window_name;

  void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);

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

    m_deviceId(0)
  {
    window_name = std::string("april_tags_output");
  }

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
  /**
   * Normalize angle to be within the interval [-pi,pi].
   */
  inline double standardRad(double t);

  void getTransformInfo(AprilTags::TagDetection& detection, double& x, double& y, double& z,
                       double& roll, double& pitch, double& yaw);
}; // AprilTagReader
