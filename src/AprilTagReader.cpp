#include "AprilTagReader.h"

const std::string intro = "\n"
    "April tags test code\n"
    "(C) 2012-2013 Massachusetts Institute of Technology\n"
    "Michael Kaess\n"
    "\n";


// default constructor
AprilTagReader::AprilTagReader(ros::NodeHandle nh) :
  // default settings, most can be modified through command line options (see below)
  m_tagDetector(NULL),
  m_tagCodes(AprilTags::tagCodes36h11),

  m_draw(false),
  m_width(640),
  m_height(480),
  m_tagSize(0.16992),
  m_fx(554.256),
  m_fy(m_fx),
  m_px(m_width/2),
  m_py(m_height/2),

  hasNewImage(false),
  m_it(nh),

  // m_image_topic("camera/rgb/image_color"),
  m_image_topic("camera/image_raw"),
  m_image_frame("camera_link")
{
  processParams(nh);
  setup();

  if (m_draw)
    window_name = std::string("april_tags_output");

  m_image_sub= m_it.subscribe(m_image_topic.c_str(), 1, &AprilTagReader::imageCallback, this);
}

AprilTagReader::~AprilTagReader()
{
  delete m_tagDetector;
}

// parse command line options to change default behavior
void AprilTagReader::processParams(ros::NodeHandle nh) {
  ROS_INFO("Ported to ROS by Shawn Hanna, using code from:\n%s", intro.c_str());

  std::string tagFamily = "36h11";
  if (nh.getParam("april_tags/tagFamily", tagFamily))
  {
    if (tagFamily=="16h5") {
      m_tagCodes = AprilTags::tagCodes16h5;
    } else if (tagFamily=="25h7") {
      m_tagCodes = AprilTags::tagCodes25h7;
    } else if (tagFamily=="25h9") {
      m_tagCodes = AprilTags::tagCodes25h9;
    } else if (tagFamily=="36h9") {
      m_tagCodes = AprilTags::tagCodes36h9;
    } else if (tagFamily=="36h11") {
      m_tagCodes = AprilTags::tagCodes36h11;
    } else {
      ROS_ERROR("Invalid tag family specified");
      exit(1);
    }
    ROS_INFO("Set tag family to: %s", tagFamily.c_str());
  }

  if (nh.getParam("april_tags/imageTopic", m_image_topic))
  {
    ROS_INFO("Listening to color images coming on the topic: %s", m_image_topic.c_str());
  }

  if (nh.getParam("april_tags/imageFrame", m_image_frame))
  {
    ROS_INFO("Frame that the image is a child of: %s", m_image_frame.c_str());
  }

  if (nh.getParam("april_tags/draw", m_draw))
  {
    if (m_draw)
      ROS_INFO("Drawing has been enabled. Detected tags will be shown in a separate window");
    else
      ROS_INFO("Separate April Tag detection window will not be shown. Results are viewable in rviz");
  }
  else
  {
      ROS_INFO("Not showing the april detections tags onscreen");
  }

  if (nh.getParam("april_tags/height", m_height))
  {
    ROS_INFO("Height of image set to: %d", m_height);
  }

  if (nh.getParam("april_tags/width", m_width))
  {
    m_px = m_width/2;
    ROS_INFO("Width of image set to: %d", m_width);
  }

  if (nh.getParam("april_tags/focalLength", m_fx))
  {
    m_fy = m_fx;
    ROS_INFO("Focal length set to: %lf", m_fx);
  }

  if (nh.getParam("april_tags/tagSize", m_tagSize))
  {
    ROS_INFO("Tag size (square black frame) in meters, set to: %lf", m_tagSize);
  }
}

void AprilTagReader::setup() {
  m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

  // prepare window for drawing the camera images
  if (m_draw) {
    cv::namedWindow(window_name, 1);
  }
}

void AprilTagReader::print_detection(AprilTags::TagDetection& detection) {
  stringstream ss;
  ss << "  Id: " << detection.id << " (Hamming: " << detection.hammingDistance << ")";

  // recovering the relative pose of a tag:

  // NOTE: for this to be accurate, it is necessary to use the
  // actual camera parameters here as well as the actual tag size
  // (m_fx, m_fy, m_px, m_py, m_tagSize)

  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;
  detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                           translation, rotation);

  Eigen::Matrix3d F;
  F <<
    1, 0,  0,
    0,  -1,  0,
    0,  0,  1;
  Eigen::Matrix3d fixed_rot = F*rotation;
  double yaw, pitch, roll;
  wRo_to_euler(fixed_rot, yaw, pitch, roll);

  ss << "  distance=" << translation.norm()
       << "m, x=" << translation(0)
       << ", y=" << translation(1)
       << ", z=" << translation(2)
       << ", yaw=" << yaw
       << ", pitch=" << pitch
       << ", roll=" << roll
       << endl;

  ROS_DEBUG_STREAM(ss.str());
  // Also note that for SLAM/multi-view application it is better to
  // use reprojection error of corner points, because the noise in
  // this relative pose is very non-Gaussian; see iSAM source code
  // for suitable factors.
}

// The processing function where m_images are retrieved, tags detected,
// and information about detections generated
void AprilTagReader::read() {
  // detect April tags (requires a gray scale m_image)
  if (hasNewImage)
  {
    cv::cvtColor(m_image, m_image_gray, CV_BGR2GRAY);

    m_lastReadTags = m_tagDetector->extractTags(m_image_gray);
    hasNewImage = false;

    // print out each detection
    //cout << m_lastReadTags.size() << " tags detected:" << endl;
    for (int i=0; i<m_lastReadTags.size(); i++) {
      print_detection(m_lastReadTags[i]);
    }

    // show the current m_image including any m_lastReadTags
    if (m_draw) {
      for (int i=0; i<m_lastReadTags.size(); i++) {
        // also highlight in the m_image
        m_lastReadTags[i].draw(m_image);
      }
      ROS_INFO("Displaying image");
      imshow(window_name, m_image); // OpenCV call
    }
  }
}

void AprilTagReader::wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
  yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
  double c = cos(yaw);
  double s = sin(yaw);
  pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
  roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double AprilTagReader::standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

void AprilTagReader::getTransformInfo(AprilTags::TagDetection& detection,
                                     double& x, double& y,double& z,
                                     double& roll, double& pitch,double& yaw){
  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;
  detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                           translation, rotation);

  Eigen::Matrix3d F;
  F <<
    1, 0,  0,
    0,  -1,  0,
    0,  0,  1;
  Eigen::Matrix3d fixed_rot = F*rotation;
  wRo_to_euler(fixed_rot, yaw, pitch, roll);

  x = translation(0);
  y = translation(1);
  z = translation(2);
}

void AprilTagReader::imageCallback(const sensor_msgs::ImageConstPtr& img)
{
  //ROS_INFO("Got new image");

  //If hasNewImage is true, then the system SHOULD be processing an image, extracting
  // the april tags if any exist
  if (!hasNewImage)
  {
    m_lastImageReceivedTime = ros::Time::now();

    // cv_bridge is a way to convert ros images to opencv format images
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, std::string());
      m_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    hasNewImage = true;
  }
}
