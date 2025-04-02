#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# 该示例代码展示KINOVA机械臂完整运动控制，包含以下功能：
# 1. 复位（Home）动作
# 2. 绝对位置运动（笛卡尔空间和关节空间）
# 3. 相对位置运动
# 4. 夹爪控制
# 5. 预定义路径轨迹运动
###

import sys
import rospy
import time

from kortex_driver.srv import *
from kortex_driver.msg import *
class ExampleFullArmMovement:
    def __init__(self):
        try:
            # 初始化ROS节点
            rospy.init_node('example_full_arm_movement_python')

            # 归位动作的预设ID
            self.HOME_ACTION_IDENTIFIER = 2

            # 获取ROS参数
            self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            # 初始化动作主题订阅器
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # 初始化所有需要的服务代理
            # 清除错误服务
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            # 读取动作服务
            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            # 执行动作服务
            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            # 设置笛卡尔参考系服务
            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            # 夹爪控制服务
            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            # 动作通知服务
            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
      
            # 获取产品配置服务
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            # 路径验证服务
            validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    # 动作通知回调函数
    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event
  
    # 笛卡尔路径点生成函数
    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        """
        创建笛卡尔空间路径点
        参数说明：
        new_x, new_y, new_z - 绝对坐标（米）
        new_theta_x/y/z - 末端姿态欧拉角（度）
        blending_radius - 混合半径（米），用于路径平滑
        """
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        # 设置目标位置和姿态
        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
      
        # 使用基坐标系
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius  # 混合半径
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

        return waypoint

    # 动作执行状态监控
    def wait_for_action_end_or_abort(self):
        """ 持续监测动作执行状态，直到完成或中止 """
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    # 示例：订阅机器人通知
    def example_subscribe_to_a_robot_notification(self):
        """ 启用动作状态通知 """
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

    # 示例：清除错误
    def example_clear_faults(self):
        """ 清除所有系统错误 """
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    # 示例：机械臂归位
    def example_home_the_robot(self):
        """ 执行预定义的归位动作（绝对位置运动） """
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER  # 归位动作ID固定为2
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

    # 示例：设置笛卡尔参考系
    def example_set_cartesian_reference_frame(self):
        """ 设置笛卡尔坐标系为混合坐标系 """
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

    # 示例：笛卡尔相对运动（Z轴提升0.1米）
    def example_send_cartesian_pose(self):
        """ 相对当前位置进行Z轴提升 """
        self.last_action_notif_type = None
        # 获取当前绝对位置
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        # 在当前位置基础上Z轴+0.1米（相对运动）
        trajectory.waypoints.append(
            self.FillCartesianWaypoint(
                feedback.base.commanded_tool_pose_x,  # X保持
                feedback.base.commanded_tool_pose_y,  # Y保持 
                feedback.base.commanded_tool_pose_z + 0.10,  # Z增加0.1米
                feedback.base.commanded_tool_pose_theta_x,  # 姿态保持
                feedback.base.commanded_tool_pose_theta_y,
                feedback.base.commanded_tool_pose_theta_z,
                0)  # 精确到达
        )

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        # 执行运动
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    # 示例：关节空间绝对运动（回到垂直姿态）
    def example_send_joint_angles(self):
        """ 所有关节运动到0度位置（绝对角度） """
        self.last_action_notif_type = None

        req = ExecuteActionRequest()
        trajectory = WaypointList()
        waypoint = Waypoint()
        angularWaypoint = AngularWaypoint()

        # 设置所有关节角度为0度
        for _ in range(self.degrees_of_freedom):
            angularWaypoint.angles.append(0.0)

        # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded. 
        # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
        angular_duration = 0
        angularWaypoint.duration = angular_duration

        # Initialize Waypoint and WaypointList
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(angularWaypoint)
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)

        # 验证轨迹有效性
        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False

        # 自动调整运动时间
        error_number = len(res.output.trajectory_error_report.trajectory_error_elements)
        MAX_ANGULAR_DURATION = 30
        angular_duration = 0
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
            # It should be possible to reach position within 30s
            # WaypointList is invalid (other error than angularWaypoint duration)
            rospy.loginfo("WaypointList is invalid")
            return False

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
      
        # 执行运动
        rospy.loginfo("Sending the robot vertical...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    # 示例：夹爪控制
    def example_send_gripper_command(self, value):
        """ 控制夹爪开合 
        value: 0.0-1.0对应全开到全闭 """
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value  # 设置开合程度
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

    # 示例：预定义笛卡尔绝对路径
    def example_cartesian_waypoint_action(self):
        """ 执行预定义的绝对路径轨迹 """
        self.last_action_notif_type = None

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        config = self.get_product_configuration()

        if config.output.model == ModelId.MODEL_ID_L31:
            # Gen3 Lite型号的路径点
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.439, 0.194, 0.448, 90.6, -1.0, 150, 0))  # 点1
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.200, 0.150, 0.400, 90.6, -1.0, 150, 0))  # 点2
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.350, 0.050, 0.300, 90.6, -1.0, 150, 0))  # 点3
        else:
            # 标准Gen3型号的路径点
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7, 0.0, 0.5, 90, 0, 90, 0))    # 起始点
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7, 0.0, 0.33, 90, 0, 90, 0.1))  # 下降点
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7, 0.48, 0.33, 90, 0, 90, 0.1)) # 右移点
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.61, 0.22, 0.4, 90, 0, 90, 0.1))# 斜向点
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7, 0.48, 0.33, 90, 0, 90, 0.1)) # 返回右移点
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.63, -0.22, 0.45, 90, 0, 90, 0.1))# 左移点
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.65, 0.05, 0.45, 90, 0, 90, 0))  # 结束点
      
        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
      
        # 执行路径
        rospy.loginfo("Executing Kortex action ExecuteWaypointTrajectory...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call action ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def main(self):
        # 初始化成功检查
        if not self.is_init_success:
            rospy.logerr("初始化失败")
            return

        success = True
      
        # 1. 清除错误状态
        success &= self.example_clear_faults()
      
        # 2. 启用动作通知
        success &= self.example_subscribe_to_a_robot_notification()
      
        # 3. 首次归位动作（绝对位置）
        success &= self.example_home_the_robot()
      
        # 4. 夹爪全开（0.0为全开）
        if self.is_gripper_present:
            success &= self.example_send_gripper_command(0.0)
      
        # 5. 设置混合坐标系（为后续笛卡尔运动做准备）
        success &= self.example_set_cartesian_reference_frame()
      
        # 6. 执行Z轴相对运动（提升0.1米）
        success &= self.example_send_cartesian_pose()
      
        # 7. 关节空间绝对运动（各关节回零形成垂直姿态）
        success &= self.example_send_joint_angles()
      
        # 8. 夹爪半闭（0.5为50%闭合）
        if self.is_gripper_present:
            success &= self.example_send_gripper_command(0.5)
      
        # 9. 二次归位（确保回到安全位置）
        success &= self.example_home_the_robot()
      
        # 10. 执行预定义绝对路径（型号差异化轨迹）
        success &= self.example_cartesian_waypoint_action()
      
        # 11. 最终归位（完成所有操作后复位）
        success &= self.example_home_the_robot()

        # 测试结果记录
        rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)
        if not success:
            rospy.logerr("部分操作执行失败")
if __name__ == "__main__":
    ex = ExampleFullArmMovement()
    ex.main()