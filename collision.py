import numpy as np
import math

class CollisionChecker:
    def __init__(self, dh_params):
        self.dh_params = dh_params
        # [radius, length] for each link
        self.link_geometries = [
            [0.05, 0.2433],  # Link 1 (d = 243.3mm)
            [0.05, 0.01],    # Link 2 (a = 10mm)
            [0.05, 0.28],    # Link 3 (d = 280mm)
            [0.05, 0.245],   # Link 4 (a = 245mm)
            [0.05, 0.057],   # Link 5 (a = 57mm)
            [0.05, 0.235],   # Link 6 (a = 235mm)
        ]

    def get_link_transforms(self, joint_angles):
        """计算每个连杆的变换矩阵"""
        transforms = []
        T = np.eye(4)
        for i in range(len(joint_angles)):
            theta = joint_angles[i] + self.dh_params[i][0]  # 添加DH参数中的theta偏移
            d = self.dh_params[i][1]
            a = self.dh_params[i][2]
            alpha = self.dh_params[i][3]
            
            # DH变换
            ct, st = np.cos(theta), np.sin(theta)
            ca, sa = np.cos(alpha), np.sin(alpha)
            T_i = np.array([
                [ct, -st*ca,  st*sa, a*ct],
                [st,  ct*ca, -ct*sa, a*st],
                [0,   sa,     ca,    d],
                [0,   0,      0,     1]
            ])
            T = T @ T_i
            transforms.append(T.copy())
        return transforms

    def check_link_collision(self, p1, p2, p3, p4, r1, r2):
        """检查两个圆柱体之间是否碰撞"""
        # 计算两线段间最短距离
        u = p2 - p1
        v = p4 - p3
        w = p1 - p3
        
        a = np.dot(u, u)
        b = np.dot(u, v)
        c = np.dot(v, v)
        d = np.dot(u, w)
        e = np.dot(v, w)
        
        # 参数化距离
        if abs(a*c - b*b) < 1e-7:  # 平行线段
            sc = 0
            if c != 0:
                tc = e/c
            else:
                tc = 0
        else:
            sc = (b*e - c*d) / (a*c - b*b)
            tc = (a*e - b*d) / (a*c - b*b)
        
        # 确保参数在[0,1]范围内
        sc = np.clip(sc, 0, 1)
        tc = np.clip(tc, 0, 1)
        
        # 计算最近点
        p_c1 = p1 + sc * u
        p_c2 = p3 + tc * v
        
        # 检查最短距离是否小于两个圆柱体半径之和
        min_dist = np.linalg.norm(p_c1 - p_c2)
        return min_dist < (r1 + r2)

    def self_collision(self, joint_angles):
        """检查机器人是否发生自碰撞"""
        transforms = self.get_link_transforms(joint_angles)
        
        # 检查每对连杆之间的碰撞
        for i in range(len(transforms)-2):
            for j in range(i+2, len(transforms)):
                # 获取连杆的起点和终点
                p1 = transforms[i][:3, 3]
                p2 = transforms[i+1][:3, 3]
                p3 = transforms[j][:3, 3]
                p4 = transforms[j+1][:3, 3] if j < len(transforms)-1 else transforms[j][:3, 3]
                
                # 获取连杆的半径
                r1 = self.link_geometries[i][0]
                r2 = self.link_geometries[j][0]
                
                if self.check_link_collision(p1, p2, p3, p4, r1, r2):
                    print(f"Self collision detected between link {i+1} and link {j+1}")
                    return True
                    
        return False

# 为了兼容性保留原函数
def is_in_collision(joint_angles):
    return False
