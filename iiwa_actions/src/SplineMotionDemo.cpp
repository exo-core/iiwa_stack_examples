/**
 * Copyright (C) 2018 Arne Peters - arne.peters@tum.de
 * Technische Universität München
 * Chair for Robotics, Artificial Intelligence and Embedded Systems
 * Fakultät für Informatik / I6, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://www6.in.tum.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>
#include <iiwa_msgs/MoveAlongSplineAction.h>
#include <iiwa_msgs/SetPTPJointSpeedLimits.h>
#include <iiwa_msgs/SetPTPCartesianSpeedLimits.h>
#include <iiwa_msgs/SetEndpointFrame.h>

static iiwa_msgs::SplineSegment getSplineSegment (const double x, const double y, const double z, int type = iiwa_msgs::SplineSegment::SPL) {
	iiwa_msgs::SplineSegment segment;
	// Segment type
	segment.type = type;
	// Header
	segment.point.poseStamped.header.frame_id = "iiwa_link_0";
	// Pose
	segment.point.poseStamped.pose.position.x = x;
	segment.point.poseStamped.pose.position.y = y;
	segment.point.poseStamped.pose.position.z = z;
	// Orientation
	segment.point.poseStamped.pose.orientation.x = 1.03992484845e-05;
	segment.point.poseStamped.pose.orientation.y = 0.999994754791;
	segment.point.poseStamped.pose.orientation.z = 1.49843574439e-05;
	segment.point.poseStamped.pose.orientation.w = -0.00323369763346;
	// Redundancy
	segment.point.redundancy.status = -1;
	segment.point.redundancy.turn = -1;

	return segment;
}

static bool setPTPJointSpeedLimits(ros::NodeHandle& nh) {
	ROS_INFO("Setting PTP joint speed limits...");
	ros::ServiceClient setPTPJointSpeedLimitsClient = nh.serviceClient<iiwa_msgs::SetPTPJointSpeedLimits>("/iiwa/configuration/setPTPJointLimits");
	iiwa_msgs::SetPTPJointSpeedLimits jointSpeedLimits;
	jointSpeedLimits.request.joint_relative_velocity = 0.2;
	jointSpeedLimits.request.joint_relative_acceleration = 0.5;
	if (!setPTPJointSpeedLimitsClient.call(jointSpeedLimits)) {
		ROS_ERROR("Service call failed.");
		return false;
	}
	else if (!jointSpeedLimits.response.success) {
		ROS_ERROR_STREAM("Service call returned error: "+jointSpeedLimits.response.error);
		return false;
	}

	ROS_INFO("Done.");
	return true;
}

static bool setPTPCartesianSpeedLimits(ros::NodeHandle& nh) {
	ROS_INFO("Setting PTP Cartesian speed limits...");
	ros::ServiceClient setPTPCartesianSpeedLimitsClient = nh.serviceClient<iiwa_msgs::SetPTPCartesianSpeedLimits>("/iiwa/configuration/setPTPCartesianLimits");
	iiwa_msgs::SetPTPCartesianSpeedLimits cartesianSpeedLimits;
	cartesianSpeedLimits.request.maxCartesianVelocity = 0.5;
	cartesianSpeedLimits.request.maxCartesianAcceleration = 0.5;
	cartesianSpeedLimits.request.maxCartesianJerk = -1.0; // ignore
	cartesianSpeedLimits.request.maxOrientationVelocity = 0.5;
	cartesianSpeedLimits.request.maxOrientationAcceleration = 0.5;
	cartesianSpeedLimits.request.maxOrientationJerk = -1.0; // ignore
	if (!setPTPCartesianSpeedLimitsClient.call(cartesianSpeedLimits)) {
		ROS_ERROR("Failed.");
		return false;
	}
	else if (!cartesianSpeedLimits.response.success) {
		ROS_ERROR_STREAM("Service call returned error: "+cartesianSpeedLimits.response.error);
		return false;
	}

	ROS_INFO("Done.");
	return true;
}

static bool setEndpointFrame(ros::NodeHandle& nh, std::string frameId = "iiwa_link_ee") {
	ROS_INFO_STREAM("Setting endpoint frame to \""<<frameId<<"\"...");
	ros::ServiceClient setEndpointFrameClient = nh.serviceClient<iiwa_msgs::SetEndpointFrame>("/iiwa/configuration/setEndpointFrame");
	iiwa_msgs::SetEndpointFrame endpointFrame;
	endpointFrame.request.frame_id = frameId;
	if (!setEndpointFrameClient.call(endpointFrame)) {
		ROS_ERROR("Failed.");
		return false;
	}
	else if (!endpointFrame.response.success) {
		ROS_ERROR_STREAM("Service call returned error: "+endpointFrame.response.error);
		return false;
	}

	ROS_INFO("Done.");
	return true;
}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "spline_motion_demo");
	ros::NodeHandle nh;

	// Set speed limit for motions in joint coordinates
	if (!setPTPJointSpeedLimits(nh)) {
		return 1;
	}

	// Set speed limits for motions in cartesian coordinates
	if (!setPTPCartesianSpeedLimits(nh)) {
		return 1;
	}

	// Set endpoint frame to flange, so that our Cartesian target coordinates are tool independent
	if (!setEndpointFrame(nh)) {
		return 1;
	}

	// Create the action clients
	// Passing "true" causes the clients to spin their own threads
	actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction> jointPositionClient("/iiwa/action/move_to_joint_position", true);
	actionlib::SimpleActionClient<iiwa_msgs::MoveAlongSplineAction> splineMotionClient("/iiwa/action/move_along_spline", true);

	ROS_INFO("Waiting for action servers to start...");
	// Wait for the action servers to start
	jointPositionClient.waitForServer(); //will wait for infinite time
	splineMotionClient.waitForServer();

	ROS_INFO("Action server started, moving to start pose...");
	// Define a goal
	iiwa_msgs::MoveToJointPositionGoal jointPositionGoal;
	jointPositionGoal.joint_position.position.a1 =  0.00;
	jointPositionGoal.joint_position.position.a2 =  0.19;
	jointPositionGoal.joint_position.position.a3 =  0.00;
	jointPositionGoal.joint_position.position.a4 = -1.40;
	jointPositionGoal.joint_position.position.a5 =  0.00;
	jointPositionGoal.joint_position.position.a6 =  1.56;
	jointPositionGoal.joint_position.position.a7 =  0.00;
	// Send goal to action server
	jointPositionClient.sendGoal(jointPositionGoal);

	// Wait for the action to finish
	bool finished_before_timeout = jointPositionClient.waitForResult(ros::Duration(60.0));

	if (!finished_before_timeout) {
		ROS_WARN("iiwa motion timed out - exiting...");
		return 0;
	}
	else if (!jointPositionClient.getResult()->success) {
		ROS_ERROR("Action execution failed - exiting...");
		return 0;
	}

	ROS_INFO_STREAM("Executing spline motion...");
	iiwa_msgs::MoveAlongSplineGoal splineMotion;

	// Current position (manually read from /iiwa/state/CartesianPose on our iiwa 14):
	const double x = 0.478509765292;
	const double y = 0;
	const double z = 0.613500561539;

	// Set up a spline segment that we use as a base for all motion points
	// The motion will look similar to this:
	//
	//   (5)---> (1) ----(2)
	//     \             /
	//      \           /
	//       \         /
	//       (4)     (3)
	//         \     /
	//          `---´
	//

	// 1. Add current position as first point.
	// This is not mandatory but ensures that the first linear segment looks exactly as we want it to
	splineMotion.spline.segments.push_back(getSplineSegment(x, y, z, iiwa_msgs::SplineSegment::SPL));

	// 2. Move linear to the side
	splineMotion.spline.segments.push_back(getSplineSegment(x, y+0.1, z, iiwa_msgs::SplineSegment::LIN));

	// 3. Move diagonally down, following a line
	splineMotion.spline.segments.push_back(getSplineSegment(x, y+0.05, z-0.1, iiwa_msgs::SplineSegment::LIN));

	// 4. Do a bow
	splineMotion.spline.segments.push_back(getSplineSegment(x, y-0.05, z-0.1, iiwa_msgs::SplineSegment::SPL));

	// 5. Move diagonally up
	splineMotion.spline.segments.push_back(getSplineSegment(x, y-0.1, z, iiwa_msgs::SplineSegment::LIN));

	// 6. Do Linear motion back to start pose
	splineMotion.spline.segments.push_back(getSplineSegment(x, y, z, iiwa_msgs::SplineSegment::LIN));

	 // Execute motion
	splineMotionClient.sendGoal(splineMotion);
	splineMotionClient.waitForResult();

	ROS_INFO("Done.");

	//exit
	return 0;
}