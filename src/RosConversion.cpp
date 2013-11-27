
#include "AprilTags/RosConversion.h"

tf::Pose RosConversion::GetPoseFromTagDetection(AprilTags::TagDetection& detection, AprilTagReader& reader) const {
  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;

  detection.getRelativeTranslationRotation(reader.m_tagSize, reader.m_fx, reader.m_fy, reader.m_px, reader.m_py,
                                           translation, rotation);

  tf::Pose pose;

  tf::Matrix3x3 tfRot;
  matrixEigenToTF(rotation, tfRot);
  tf::Vector3 tfTrans;
  vectorEigenToTF(translation, tfTrans);

  tfRot.getRotation(pose.quaternion);
  pose.point = tfTrans;

  return pose;

  /*
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
  */

}

