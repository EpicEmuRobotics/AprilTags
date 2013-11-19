#include "AprilTagReader.h"

const string usage = "\n"
  "Usage:\n"
  "  apriltags_demo [OPTION...] [deviceID]\n"
  "\n"
  "Options:\n"
  "  -h  -?          Show help options\n"
  "  -a              Arduino (send tag ids over serial port)\n"
  "  -d              disable graphics\n"
  "  -C <bbxhh>      Tag family (default 36h11)\n"
  "  -F <fx>         Focal length in pixels\n"
  "  -W <width>      Image width (default 640, availability depends on camera)\n"
  "  -H <height>     Image height (default 480, availability depends on camera)\n"
  "  -S <size>       Tag size (square black frame) in meters\n"
  "  -E <exposure>   Manually set camera exposure (default auto; range 0-10000)\n"
  "  -G <gain>       Manually set camera gain (default auto; range 0-255)\n"
  "  -B <brightness> Manually set the camera brightness (default 128; range 0-255)\n"
  "\n";

const string intro = "\n"
    "April tags test code\n"
    "(C) 2012-2013 Massachusetts Institute of Technology\n"
    "Michael Kaess\n"
    "\n";

// changing the tag family
void AprilTagReader::setTagCodes(string s) {
  if (s=="16h5") {
    m_tagCodes = AprilTags::tagCodes16h5;
  } else if (s=="25h7") {
    m_tagCodes = AprilTags::tagCodes25h7;
  } else if (s=="25h9") {
    m_tagCodes = AprilTags::tagCodes25h9;
  } else if (s=="36h9") {
    m_tagCodes = AprilTags::tagCodes36h9;
  } else if (s=="36h11") {
    m_tagCodes = AprilTags::tagCodes36h11;
  } else {
    cout << "Invalid tag family specified" << endl;
    exit(1);
  }
}

// parse command line options to change default behavior
void AprilTagReader::parseOptions(int argc, char* argv[]) {
  int c;
  while ((c = getopt(argc, argv, ":h?adC:F:H:S:W:E:G:B:")) != -1) {
    // Each option character has to be in the string in getopt();
    // the first colon changes the error character from '?' to ':';
    // a colon after an option means that there is an extra
    // parameter to this option; 'W' is a reserved character
    switch (c) {
    case 'h':
    case '?':
      cout << intro;
      cout << usage;
      exit(0);
      break;
    case 'a':
      m_arduino = true;
      break;
    case 'd':
      m_draw = false;
      break;
    case 'C':
      setTagCodes(optarg);
      break;
    case 'F':
      m_fx = atof(optarg);
      m_fy = m_fx;
      break;
    case 'H':
      m_height = atoi(optarg);
      m_py = m_height/2;
       break;
    case 'S':
      m_tagSize = atof(optarg);
      break;
    case 'W':
      m_width = atoi(optarg);
      m_px = m_width/2;
      break;
    case 'E':
#ifndef EXPOSURE_CONTROL
      cout << "Error: Exposure option (-E) not available" << endl;
      exit(1);
#endif
      m_exposure = atoi(optarg);
      break;
    case 'G':
#ifndef EXPOSURE_CONTROL
      cout << "Error: Gain option (-G) not available" << endl;
      exit(1);
#endif
      m_gain = atoi(optarg);
      break;
    case 'B':
#ifndef EXPOSURE_CONTROL
      cout << "Error: Brightness option (-B) not available" << endl;
      exit(1);
#endif
      m_brightness = atoi(optarg);
      break;
    case ':': // unknown option, from getopt
      cout << intro;
      cout << usage;
      exit(1);
      break;
    }
  }

  if (argc == optind + 1) {
    m_deviceId = atoi(argv[optind]);
  }
}

void AprilTagReader::setup() {
  m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

#ifdef EXPOSURE_CONTROL
  // manually setting camera exposure settings; OpenCV/v4l1 doesn't
  // support exposure control; so here we manually use v4l2 before
  // opening the device via OpenCV; confirmed to work with Logitech
  // C270; try exposure=20, gain=100, brightness=150

  string video_str = "/dev/video0";
  video_str[10] = '0' + m_deviceId;
  int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

  if (m_exposure >= 0) {
    // not sure why, but v4l2_set_control() does not work for
    // V4L2_CID_EXPOSURE_AUTO...
    struct v4l2_control c;
    c.id = V4L2_CID_EXPOSURE_AUTO;
    c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
    if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
      cout << "Failed to set... " << strerror(errno) << endl;
    }
    cout << "exposure: " << m_exposure << endl;
    v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
  }
  if (m_gain >= 0) {
    cout << "gain: " << m_gain << endl;
    v4l2_set_control(device, V4L2_CID_GAIN, m_gain*256);
  }
  if (m_brightness >= 0) {
    cout << "brightness: " << m_brightness << endl;
    v4l2_set_control(device, V4L2_CID_BRIGHTNESS, m_brightness*256);
  }
  v4l2_close(device);
#endif 

  // find and open a USB camera (built in laptop camera, web cam etc)
  m_cap = cv::VideoCapture(m_deviceId);
      if(!m_cap.isOpened()) {
    cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
    exit(1);
  }
  m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
  m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
  cout << "Camera successfully opened (ignore error messages above...)" << endl;
  cout << "Actual resolution: "
       << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
       << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

  // prepare window for drawing the camera images
  if (m_draw) {
    cv::namedWindow(window_name, 1);
  }
}

void AprilTagReader::print_detection(AprilTags::TagDetection& detection) {
  cout << "  Id: " << detection.id
       << " (Hamming: " << detection.hammingDistance << ")";

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

  cout << "  distance=" << translation.norm()
       << "m, x=" << translation(0)
       << ", y=" << translation(1)
       << ", z=" << translation(2)
       << ", yaw=" << yaw
       << ", pitch=" << pitch
       << ", roll=" << roll
       << endl;

  // Also note that for SLAM/multi-view application it is better to
  // use reprojection error of corner points, because the noise in
  // this relative pose is very non-Gaussian; see iSAM source code
  // for suitable factors.
}

// The processing function where m_images are retrieved, tags detected,
// and information about detections generated
void AprilTagReader::read() {

  int frame = 0;
  // capture frame
  m_cap >> m_image;

  // alternative way is to grab, then retrieve; allows for
  // multiple grab when processing below frame rate - v4l keeps a
  // number of frames buffered, which can lead to significant lag
  //      m_cap.grab();
  //      m_cap.retrieve(m_image);

  // detect April tags (requires a gray scale m_image)
  cv::cvtColor(m_image, m_image_gray, CV_BGR2GRAY);
  m_lastReadTags = m_tagDetector->extractTags(m_image_gray);

  // print out each detection
  cout << m_lastReadTags.size() << " tags detected:" << endl;
  for (int i=0; i<m_lastReadTags.size(); i++) {
    print_detection(m_lastReadTags[i]);
  }

  // show the current m_image including any m_lastReadTags
  if (m_draw) {
    for (int i=0; i<m_lastReadTags.size(); i++) {
      // also highlight in the m_image
      m_lastReadTags[i].draw(m_image);
    }
    imshow(window_name, m_image); // OpenCV call
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
