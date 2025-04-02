import numpy as np

def gen3_lite_fwd_kinematics(theta1, theta2, theta3, theta4, theta5, theta6):
    # DH参数
    alpha = np.array([0, np.pi/2, np.pi, np.pi/2, np.pi/2, -np.pi/2])  # 每两个系之间的旋转轴方向的变化
    a = np.array([0, 0, 280, 0, 0, 0])  # 每两个系之间的链接长度
    d = np.array([128.3+115, 30, 20, 140+105, 28.5+28.5, 105+130])  # 每两个系之间的垂直距离
    theta = [theta1, theta2, theta3-np.pi, theta4-np.pi/2, theta5, theta6-np.pi/2]  # 角度全部使用弧度

    # 初始化转换矩阵
    T = [np.eye(4)]

    # 计算每两个系之间的转换矩阵
    for i in range(1, 7):
        # 根据DH参数计算旋转矩阵和平移矩阵
        c_theta_i = np.cos(theta[i-1])
        s_theta_i = np.sin(theta[i-1])
        c_alpha_i_minus_1 = np.cos(alpha[i-1])
        s_alpha_i_minus_1 = np.sin(alpha[i-1])
    
        T_temp = np.array([[c_theta_i, -s_theta_i, 0, a[i-1]],
                           [s_theta_i * c_alpha_i_minus_1, c_theta_i * c_alpha_i_minus_1, - s_alpha_i_minus_1, -s_alpha_i_minus_1 * d[i-1]],
                           [s_theta_i*s_alpha_i_minus_1, c_theta_i*s_alpha_i_minus_1, c_alpha_i_minus_1,c_alpha_i_minus_1* d[i-1]],
                           [0, 0, 0, 1]])
    
        # 更新转换矩阵
        T.append(np.dot(T[-1], T_temp))

    # 计算工具坐标系在基座坐标系中的坐标和方向
    tool_coord = T[-1][:3, 3]
    tool_orientation = T[-1][:3, :3]

    return tool_coord, tool_orientation

# 示例输入
theta = [0, 0, 0, 0, 0, 0]  # 所有theta为0
tool_coord, tool_orientation = gen3_lite_fwd_kinematics(*theta)
print("Tool Coordinates:", tool_coord)
print("Tool Orientation:")
print(tool_orientation)