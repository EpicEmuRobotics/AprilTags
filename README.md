AprilTags
=========

April Tag ROS node


For this project, you need to adjust the focal length (in pixels) and width of the image.  Converting to focal length in pixels is easy if you have both the image sensor's size and the focal length. The equation to get focal length in pixels is as follows:

focal length in pixels = (image width in pixels) * (focal length in mm) / (CCD width in mm)

For the kinect, we know the focal length in pixels to be: 525

http://wiki.ros.org/kinect_calibration/technical


For the Logitech C270 webcam, the focal length is 4mm.  However, we don't know the image size, so we have to estimate the focal length in pixels.  To do this, I tested the output from the tag at a constant distance, varying the focal length, and found that the focal length in pixels is around 790 pixels.

http://logitech-en-amr.custhelp.com/app/answers/detail/a_id/17556
http://www.dpreview.com/glossary/camera-system/sensor-sizes
http://www.siliconimaging.com/Lens%20Image%20formats.htm


For the camera in Gazebo, we can calculate the focal length via the formula: focal_lengtth = image_width / (2*tan(hfov_radian / 2).  In our case, we have a width of 640, and fov is 60 degrees (1.04719755 radians). Doing the calculation, we see the focal length in px is: 554.256

http://answers.ros.org/question/9761/incorrect-camera-extrinsics-for-simulated-gazebo-cameras/
