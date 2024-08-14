import cv2
import numpy as np

# 定义旋转向量
rvec = np.array([0.5773503*2.0943951, -0.5773503*2.0943951, 0.5773503*2.0943951], np.float32)

# 将旋转向量转换为旋转矩阵
R, _ = cv2.Rodrigues(rvec)

# 计算旋转矩阵的欧拉角
rvec_euler, _ = cv2.Rodrigues(R)

# 将弧度转换为角度
rvec_euler_deg = np.degrees(rvec_euler)

# 打印结果
print("欧拉角（弧度）：", rvec_euler)
print("欧拉角（角度）：", rvec_euler_deg)