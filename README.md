AprilTags
=========

April Tag ROS node


For this project, you need to adjust the focal length (in pixels) and width of the image.  Converting to focal length in pixels is easy if you have both the image sensor's size and the focal length. The equation to get focal length in pixels is as follows:

focal length in pixels = (image width in pixels) * (focal length in mm) / (CCD width in mm)

For the kinect, we know the focal length in pixels to be: 525

http://wiki.ros.org/kinect_calibration/technical


For the Logitech C270 webcam, the focal length is 4mm and the image sensor size is ~4.6188mm?.  This gives a focal length in pixels of 480*4E-3/4.6188E-3 which is: 415.692388

http://logitech-en-amr.custhelp.com/app/answers/detail/a_id/17556
http://www.dpreview.com/glossary/camera-system/sensor-sizes
http://www.siliconimaging.com/Lens%20Image%20formats.htm
