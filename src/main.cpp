/*
 * Copyright 2016 Delft Robotics B.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "node.hpp"

int main(int argc, char * argv[]) {
//	ROS_INFO("%d\n",CV_MAJOR_VERSION);
//	ROS_INFO("%d\n",CV_MINOR_VERSION);
	ROS_INFO("here0");
	ros::init(argc, argv, ROS_PACKAGE_NAME);
//	ros::NodeHandle n;
//	image_transport::ImageTransport it(n);
	ROS_INFO("here1");
	camera_pose_calibration::CameraPoseCalibrationNode node;
	ROS_INFO_STREAM(ROS_PACKAGE_NAME << " node initialized.");
	ros::spin();
}
