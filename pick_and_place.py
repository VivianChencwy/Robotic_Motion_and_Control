#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time

from kortex_driver.srv import *
from kortex_driver.msg import *

class SimplifiedArmMovement:
    def __init__(self):
        try:
            rospy.init_node('simplified_arm_movement')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Initialize critical services
            self.initialize_services()

        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def initialize_services(self):
        """ Maintain original service initialization """
        clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
        rospy.wait_for_service(clear_faults_full_name)
        self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

        execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

        if self.is_gripper_present:
            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_completion(self):
        """ Preserve original completion detection """
        while not rospy.is_shutdown():
            if self.last_action_notif_type == ActionEvent.ACTION_END:
                rospy.loginfo("Action completed")
                return True
            elif self.last_action_notif_type == ActionEvent.ACTION_ABORT:
                rospy.loginfo("Action aborted")
                return False
            time.sleep(0.01)

    def example_home_the_robot(self):
        """ Preserve original homing method """
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
      
        req = ExecuteActionRequest()
        req.input = res.output
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteAction")
            return False
        return self.wait_for_action_completion()

    def absolute_position_move(self, x, y, z, theta_x=90.0, theta_y=0.0, theta_z=90.0):
        """ Modified from original cartesian pose method for absolute positioning """
        self.last_action_notif_type = None
        req = ExecuteActionRequest()
        trajectory = WaypointList()

        trajectory.waypoints.append(
            self.FillCartesianWaypoint(
                x, y, z,
                theta_x, theta_y, theta_z,
                0  # No blending
            )
        )

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        try:
            self.execute_action(req)
            return self.wait_for_action_completion()
        except rospy.ServiceException:
            rospy.logerr("Absolute position move failed")
            return False

    def FillCartesianWaypoint(self, x, y, z, theta_x, theta_y, theta_z, blending_radius):
        """ Preserve original waypoint creation """
        waypoint = Waypoint()
        cartesian = CartesianWaypoint()
        cartesian.pose.x, cartesian.pose.y, cartesian.pose.z = x, y, z
        cartesian.pose.theta_x = theta_x
        cartesian.pose.theta_y = theta_y
        cartesian.pose.theta_z = theta_z
        cartesian.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesian.blending_radius = blending_radius
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesian)
        return waypoint

    def control_gripper(self, value):
        """ Preserve original gripper control """
        if not self.is_gripper_present:
            return True
          
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        try:
            self.send_gripper_command(req)
            time.sleep(0.5)
            return True
        except rospy.ServiceException:
            rospy.logerr("Gripper command failed")
            return False

    def execute_sequence(self):
        """ Main execution sequence """
        if not self.is_init_success:
            return False

        success = True
      
        # 1. Clear faults
        success &= self.clear_faults()
      
        # 2. Home position
        success &= self.example_home_the_robot()
      
        # 3. First absolute position (modify coordinates as needed)
        success &= self.absolute_position_move(0.5, 0.0, 0.4)
      
        # 4. Close gripper to 10%
        success &= self.control_gripper(0.1)
      
        # 5. Second absolute position
        success &= self.absolute_position_move(0.4, 0.3, 0.3)

        success &= self.absolute_position_move(0.4, 0.3, 0.3)
      
        # 6. Open gripper fully
        success &= self.control_gripper(0.0)
      
        # 7. Return to home
        success &= self.example_home_the_robot()

        return success

if __name__ == "__main__":
    arm = SimplifiedArmMovement()
    if arm.execute_sequence():
        rospy.loginfo("Sequence completed successfully")
    else:
        rospy.logerr("Sequence execution failed")