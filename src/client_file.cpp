#include "node.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");


  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<camera_pose_calibration::CalibrateFile>("/calibrate_file");
//  beginner_tutorials::AddTwoInts srv;
  // camera_pose_calibration::CalibrateFile::Request & req
  camera_pose_calibration::CalibrateFile srv;
  srv.request.image = "/home/walt/catkin_ws/src/camera_pose_calibration/data/intensity.png";
  srv.request.cloud = "/home/walt/catkin_ws/src/camera_pose_calibration/data/cloud.pcd";
  srv.request.camera_frame = "camera_frame"; 
  srv.request.tag_frame = "target_frame";
  srv.request.target_frame = "target_frame";
  srv.request.point_cloud_scale_x = 1;
  srv.request.point_cloud_scale_y = 1;
  srv.request.pattern.pattern_width = 3;
  srv.request.pattern.pattern_height = 9;
  srv.request.pattern.pattern_distance = 0.04;
  //this is the distance between circles of the same row or col
  srv.request.pattern.neighbor_distance = 0;//effective only if >0
  srv.request.pattern.valid_pattern_ratio_threshold = 0.8;

  if (client.call(srv))
  {
  	float q0 = srv.response.transform.rotation.x;
  	float q1 = srv.response.transform.rotation.y;
  	float q2 = srv.response.transform.rotation.z;
  	float q3 = srv.response.transform.rotation.w;
    float R20 = (q1 * q3 - q0 * q2) * 2.0f;
    float R21 = (q2 * q3 + q0 * q1) * 2.0f;
    float R22 = 1.0f - ((q1 * q1 + q2 * q2) * 2.0f);
    float R01 = (q1 * q2 - q0 * q3) * 2.0f;
    float R11 = 1.0f - ((q1 * q1 + q3 * q3) * 2.0f);
    float pitch = -asin(R20);
    float roll = atan2(R21, R22);
    float yaw = -atan2(R01, R11);
    ROS_INFO("Translation: x=%f, y=%f, z=%f\n", srv.response.transform.translation.x, srv.response.transform.translation.y, srv.response.transform.translation.z);
    // ROS_INFO("Rotation: x=%f, y=%f, z=%f, w=%f\n", srv.response.transform.rotation.x, srv.response.transform.rotation.y, srv.response.transform.rotation.z, srv.response.transform.rotation.w);
    ROS_INFO("Rotation: p=%f, r=%f, y=%f\n",pitch*57.3f,roll*57.3f,yaw*57.3f);
  }
  else
  {
    ROS_ERROR("Failed to call service calibrate_file");
    return 1;
  }

  return 0;
}
