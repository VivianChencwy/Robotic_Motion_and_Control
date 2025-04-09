#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import time
import math
import numpy as np  # ---------- 在这里引入 numpy ----------

from kortex_driver.srv import *
from kortex_driver.msg import *

# ---------- 在这里插入第一段代码的内容（正向运动学函数） ----------
def gen3_lite_fwd_kinematics(theta1, theta2, theta3, theta4, theta5, theta6):
    # DH参数
    alpha = np.array([0, np.pi/2, np.pi, np.pi/2, np.pi/2, -np.pi/2])  # 每两个系之间的旋转轴方向的变化
    a = np.array([0, 0, 280, 0, 0, 0])  # 每两个系之间的链接长度 (单位: mm)
    d = np.array([128.3+115, 30, 20, 140+105, 28.5+28.5, 105+130])  # 每两个系之间的垂直距离 (单位: mm)
    theta = [
        theta1,
        theta2 + np.pi/2,
        theta3 + np.pi/2,
        theta4 + np.pi/2,
        theta5,
        theta6 - np.pi/2
    ]  # 角度全部使用弧度

    # 初始化转换矩阵
    T = [np.eye(4)]

    # 计算每两个系之间的转换矩阵
    for i in range(1, 7):
        c_theta_i = np.cos(theta[i-1])
        s_theta_i = np.sin(theta[i-1])
        c_alpha_i_minus_1 = np.cos(alpha[i-1])
        s_alpha_i_minus_1 = np.sin(alpha[i-1])
    
        T_temp = np.array([
            [c_theta_i, -s_theta_i, 0,            a[i-1]],
            [s_theta_i * c_alpha_i_minus_1,  c_theta_i * c_alpha_i_minus_1, - s_alpha_i_minus_1, -s_alpha_i_minus_1 * d[i-1]],
            [s_theta_i * s_alpha_i_minus_1,  c_theta_i * s_alpha_i_minus_1,   c_alpha_i_minus_1,  c_alpha_i_minus_1 * d[i-1]],
            [0,                             0,                              0,                   1]
        ])
    
        T.append(np.dot(T[-1], T_temp))

    # 工具坐标系在基座坐标系中的位置 (mm)
    tool_coord = T[-1][:3, 3]
    # 工具坐标系在基座坐标系中的方向 (旋转矩阵)
    tool_orientation = T[-1][:3, :3]

    return tool_coord, tool_orientation
# ---------- 第一段代码插入结束 ----------

# ---------- 可选：将旋转矩阵转换为欧拉角，便于与第二段代码的XYZ姿态格式对接 ----------
def rot_mat_to_euler_xyz(R):
    """
    将旋转矩阵R转换为XYZ旋转顺序的欧拉角(弧度)。
    注意：此处只做一个简单示例，具体是否与机械臂的内部定义完全匹配，
    需结合实际DH和控制系统的姿态定义来确认。
    """
    # 假设R是一个3x3的旋转矩阵
    sy = math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return x, y, z


class ExampleFullArmMovement:
    def __init__(self):
        try:
            rospy.init_node('example_full_arm_movement_python')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event
    
    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

        return waypoint

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def example_subscribe_to_a_robot_notification(self):
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        else:
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        self.last_action_notif_type = None
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        try:
            self.set_cartesian_reference_frame(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        rospy.sleep(0.25)
        return True

    def example_send_cartesian_pose(self):
        self.last_action_notif_type = None
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        trajectory.waypoints.append(
            self.FillCartesianWaypoint(
                feedback.base.commanded_tool_pose_x,
                feedback.base.commanded_tool_pose_y,
                feedback.base.commanded_tool_pose_z + 0.10,
                feedback.base.commanded_tool_pose_theta_x,
                feedback.base.commanded_tool_pose_theta_y,
                feedback.base.commanded_tool_pose_theta_z,
                0)
        )

        trajectory.duration = 0
        trajectory.use_optimal_blending = False

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def example_send_cartesian(self, x, y, z, theta_x, theta_y, theta_z, blending_radius):
        self.last_action_notif_type = None
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        trajectory.waypoints.append(
            self.FillCartesianWaypoint(
                x,
                y,
                z,
                theta_x,
                theta_y,
                theta_z,
                blending_radius)
        )

        trajectory.duration = 0
        trajectory.use_optimal_blending = False

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def example_send_joint_angles(self):
        self.last_action_notif_type = None

        req = ExecuteActionRequest()

        trajectory = WaypointList()
        waypoint = Waypoint()
        angularWaypoint = AngularWaypoint()

        for _ in range(self.degrees_of_freedom):
            angularWaypoint.angles.append(0.0)

        angular_duration = 0
        angularWaypoint.duration = angular_duration

        waypoint.oneof_type_of_waypoint.angular_waypoint.append(angularWaypoint)
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)

        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False

        error_number = len(res.output.trajectory_error_report.trajectory_error_elements)
        MAX_ANGULAR_DURATION = 30

        while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION) :
            angular_duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration

            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(res.output.trajectory_error_report.trajectory_error_elements)

        if (angular_duration == MAX_ANGULAR_DURATION) :
            rospy.loginfo("WaypointList is invalid")
            return False

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        
        rospy.loginfo("Sending the robot vertical...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def example_send_gripper_command(self, value):
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    def example_cartesian_waypoint_action(self):
        self.last_action_notif_type = None

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        config = self.get_product_configuration()

        if config.output.model == ModelId.MODEL_ID_L31:
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.439,  0.194,  0.448, 90.6, -1.0, 150, 0))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.200,  0.150,  0.400, 90.6, -1.0, 150, 0))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.350,  0.050,  0.300, 90.6, -1.0, 150, 0))
        else:
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.0,   0.5,  90, 0, 90, 0))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.0,   0.33, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.48,  0.33, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.61, 0.22,  0.4,  90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.48,  0.33, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.63, -0.22, 0.45, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.65, 0.05,  0.45, 90, 0, 90, 0))
        
        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        
        rospy.loginfo("Executing Kortex action ExecuteWaypointTrajectory...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call action ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def main(self):
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/full_arm_movement_python")
        except:
            pass

        if success:
            # 清除故障
            success &= self.example_clear_faults()
            # 订阅通知
            success &= self.example_subscribe_to_a_robot_notification()
            # 回到Home位置
            success &= self.example_home_the_robot()

            # 下述原有示例保留
            success &= self.example_set_cartesian_reference_frame()
            success &= self.example_send_cartesian(0.406, 0.253, -0.019, 90, -1.0, 150, 0)

            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0.9)
                rospy.sleep(2)
            else:
                rospy.logwarn("No gripper is present on the arm.")

            success &= self.example_send_cartesian(0.515, 0.212, 0.247, 90, -1.0, 150, 0)
            success &= self.example_send_cartesian(0.471, 0.085, -0.021, 90, -1.0, 150, 0)

            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0.2)
                rospy.sleep(2)
            else:
                rospy.logwarn("No gripper is present on the arm.")

            # 回到Home位置
            success &= self.example_home_the_robot()

            # ---------- 在这里插入正向运动学的调用示例，演示根据theta1-6让机器人到达目标位姿 ----------
            theta1 = 0.0
            theta2 = 0.0
            theta3 = 0.0
            theta4 = 0.0
            theta5 = 0.0
            theta6 = 0.0

            tool_coord, tool_orientation = gen3_lite_fwd_kinematics(theta1, theta2, theta3, theta4, theta5, theta6)
            print("Tool Coordinates:", tool_coord)
            print("Tool Orientation:\n", tool_orientation)

            # 将矩阵转换为XYZ欧拉角（弧度）
            rx, ry, rz = rot_mat_to_euler_xyz(tool_orientation)
            # 转换为度
            rx_deg = rx * 180.0 / math.pi
            ry_deg = ry * 180.0 / math.pi
            rz_deg = rz * 180.0 / math.pi

            # 由于第一段代码的位移是 mm，需要转换为 m
            x_m = tool_coord[0] / 1000.0
            y_m = tool_coord[1] / 1000.0
            z_m = tool_coord[2] / 1000.0

            rospy.loginfo("Moving to FK position and orientation based on theta1-6...")
            # 这里的blending_radius设为0
            success &= self.example_send_cartesian(x_m, y_m, z_m, rx_deg, ry_deg, rz_deg, 0)

            # 再次回到Home位置
            success &= self.example_home_the_robot()
            # ---------- 插入结束 ----------

        rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")


if __name__ == "__main__":
    ex = ExampleFullArmMovement()
    ex.main()
