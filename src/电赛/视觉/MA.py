from maix import image, camera, display, app, time
import cv2
import math
import servo
import numpy as np
# 新增 GPIO 控制（假设使用 MaixPy 的 GPIO 模块）
from maix import GPIO

# 初始化 GPIO（假设使用引脚 P9 作为开关信号）
switch_pin = GPIO(29, GPIO.OUT)
switch_pin.value(0)  # 初始低电平

# 配置参数
INIT_ANGLE_X = 130    # X轴舵机初始角度(中间位置)
INIT_ANGLE_Y = 112   # Y轴舵机初始角度(合适位置)
FILTER_FACTOR = 0.15  # 滤波系数，减小抖动

KP_X = 0.114999       # X轴比例系数
KI_X = 0.0005         # X轴积分系数
KD_X = 0.2695         # X轴微分系数

KP_Y = 0.010999       # Y轴比例系数
KI_Y = 0.0010         # Y轴积分系数
KD_Y = 0.3098         # Y轴微分系数

# 1cm对应的像素阈值（基础值，会根据距离动态调整）
BASE_CM1_PIXELS = 8   # 基准位置1cm对应像素数
STOP_RANGE = BASE_CM1_PIXELS  # 停止动作的范围（1cm）
CIRCLE_RADIUS_CM = 6  # 要绘制的圆半径(cm)
CIRCLE_POINTS = 50    # 圆的点数量，控制平滑度

# 图像尺寸(与摄像头匹配) ？？？？？？？？？？？？？？？？
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IMAGE_CENTER_X = IMAGE_WIDTH // 2  # 摄像头中心X坐标
IMAGE_CENTER_Y = IMAGE_HEIGHT // 2 # 摄像头中心Y坐标

# 初始化设备（使用彩色模式）
try:
    cam = camera.Camera(320, 240, fps=10, format=image.Format.FMT_RGB888)
    disp = display.Display()
    servo_x = servo.Servo(6, INIT_ANGLE_X)  # X轴舵机(PWM6)
    servo_y = servo.Servo(7, INIT_ANGLE_Y)  # Y轴舵机(PWM7)
except Exception as e:
    print(f"设备初始化错误: {e}")
    exit(1)

# 控制变量初始化
last_err_x = 0
last_err_y = 0
integral_x = 0
integral_y = 0
derivative_x = 0  # X轴微分项
derivative_y = 0  # Y轴微分项

# 形态学运算核
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

def pid_control(current_err, last_err, integral, derivative, kp, ki, kd, max_integral=50):
    """完整的PID控制器实现"""
    integral += current_err
    integral = max(-max_integral, min(integral, max_integral))
    derivative = current_err - last_err
    output = kp * current_err + ki * integral + kd * derivative
    return output, current_err, integral, derivative

def draw_6cm_circle(img, center_x, center_y, cm1_pixels):
    """绘制距离矩形中心6cm的圆，根据当前距离动态调整像素比例"""
    if img is None:
        return
        
    # 动态计算6cm对应的像素半径
    circle_radius_pixels = CIRCLE_RADIUS_CM * cm1_pixels
    
    # 生成圆上的点
    angles = np.linspace(0, 2 * np.pi, CIRCLE_POINTS, endpoint=False)
    points = []
    
    for angle in angles:
        x = int(center_x + circle_radius_pixels * np.cos(angle))
        y = int(center_y + circle_radius_pixels * np.sin(angle))
        # 确保点在图像范围内
        x = max(0, min(img.shape[1]-1, x))
        y = max(0, min(img.shape[0]-1, y))
        points.append((x, y))
    
    # 绘制圆（连接所有点）
    for i in range(CIRCLE_POINTS):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % CIRCLE_POINTS]
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 1)

try:
    while not app.need_exit():
        # 读取彩色图像
        img = cam.read()
        if img is None:
            time.sleep(0.01)
            continue
            
        # 转换为OpenCV格式（保持彩色）
        img_raw = image.image2cv(img, copy=False)
        if img_raw is None:
            time.sleep(0.01)
            continue
        
        # 转换为HSV色彩空间，提取亮度通道进行边缘检测
        hsv = cv2.cvtColor(img_raw, cv2.COLOR_RGB2HSV)
        value_channel = hsv[:, :, 2]
        
        # 图像处理（基于亮度通道）
        blurred = cv2.bilateralFilter(value_channel, 9, 10, 10)
        closed = cv2.morphologyEx(blurred, cv2.MORPH_CLOSE, kernel)
        edged = cv2.Canny(closed, 50, 150)

        # 寻找轮廓
        contours, _ = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        target_found = False
        target_cx, target_cy = 0, 0
        current_cm1_pixels = BASE_CM1_PIXELS  # 当前1cm对应的像素数

        # 筛选长方形轮廓
        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # 检测四边形(长方形/矩形)
            if len(approx) == 4:
                area = cv2.contourArea(approx)
                # 过滤小面积和非凸多边形
                if area > 1000 and cv2.isContourConvex(approx):
                    # 计算长宽比
                    rect = cv2.minAreaRect(approx)
                    (_, _), (width, height), _ = rect
                    if min(width, height) > 0:
                        aspect_ratio = max(width, height) / min(width, height)
                        if 1.3 <= aspect_ratio <= 5:  # 长方形比例范围
                            # 计算中心点
                            M = cv2.moments(approx)
                            if M["m00"] != 0:
                                target_cx = int(M["m10"] / M["m00"])
                                target_cy = int(M["m01"] / M["m00"])
                                target_found = True
                                
                                # 根据矩形大小动态调整1cm对应的像素数
                                rect_diagonal = math.sqrt(width**2 + height**2)
                                base_diagonal = 300  # 基准对角线长度（可校准）
                                current_cm1_pixels = BASE_CM1_PIXELS * (rect_diagonal / base_diagonal)
                                current_cm1_pixels = max(2, min(20, current_cm1_pixels))  # 限制范围
                                
                                # 绘制矩形边框
                                cv2.drawContours(img_raw, [approx], 0, (0, 255, 0), 2)
                                
                                # 绘制矩形中心（红色点）
                                cv2.circle(img_raw, (target_cx, target_cy), 3, (0, 0, 255), -1)
                                
                                # 绘制图像中心参考线（蓝色十字）
                                cv2.line(img_raw, (IMAGE_CENTER_X, 0), 
                                        (IMAGE_CENTER_X, IMAGE_HEIGHT), (255, 0, 0), 1)
                                cv2.line(img_raw, (0, IMAGE_CENTER_Y), 
                                        (IMAGE_WIDTH, IMAGE_CENTER_Y), (255, 0, 0), 1)
                                
                                # 绘制1cm范围参考圆（黄色）
                                stop_range = int(current_cm1_pixels)
                                cv2.circle(img_raw, (IMAGE_CENTER_X, IMAGE_CENTER_Y), 
                                          stop_range, (0, 255, 255), 1)
                                
                                # 绘制6cm外圆（绿色）
                                draw_6cm_circle(img_raw, target_cx, target_cy, current_cm1_pixels)
                                
                                # 核心：绘制摄像头中心到矩形中心的红线（重点添加部分）
                                cv2.line(img_raw, 
                                        (IMAGE_CENTER_X, IMAGE_CENTER_Y),  # 摄像头中心
                                        (target_cx, target_cy),             # 矩形中心
                                        (0, 0, 255), 2)                    # 红色，线宽2
                                
                                break  # 只跟踪第一个符合条件的长方形

        # 控制逻辑
        if target_found:
            # 显示动态像素系数（调试用）
            cv2.putText(img_raw, f"1cm = {current_cm1_pixels:.1f}px", (10, 140), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            cv2.putText(img_raw, f"6cm圆像素半径: {int(CIRCLE_RADIUS_CM * current_cm1_pixels)}", (10, 160), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            
            # 计算偏差和距离
            distance = math.hypot(target_cx - IMAGE_CENTER_X, target_cy - IMAGE_CENTER_Y)
            err_x = IMAGE_CENTER_X - target_cx
            err_y = IMAGE_CENTER_Y - target_cy
            
            # 显示误差信息
            cv2.putText(img_raw, f"Error X: {err_x:.1f}px", (10, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(img_raw, f"Error Y: {err_y:.1f}px", (10, 100), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(img_raw, f"6cm实际距离", (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # 一阶滞后滤波
            filtered_err_x = FILTER_FACTOR * err_x + (1 - FILTER_FACTOR) * last_err_x
            filtered_err_y = FILTER_FACTOR * err_y + (1 - FILTER_FACTOR) * last_err_y

            # 判断是否在1cm范围内（使用动态调整的范围）
            if distance <= current_cm1_pixels:
                switch_pin.value(1)  # 输出高电平
                cv2.putText(img_raw, "Within 1cm - Servo STOP", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            else:
                # PID控制计算
                delta_x, last_err_x, integral_x, derivative_x = pid_control(
                    filtered_err_x, last_err_x, integral_x, derivative_x, KP_X, KI_X, KD_X
                )
                delta_y, last_err_y, integral_y, derivative_y = pid_control(
                    filtered_err_y, last_err_y, integral_y, derivative_y, KP_Y, KI_Y, KD_Y
                )

                # 计算目标角度并限制范围
                target_angle_x = servo_x.current_angle + delta_x
                target_angle_y = servo_y.current_angle + delta_y
                target_angle_x = max(0, min(180, target_angle_x))
                target_angle_y = max(0, min(180, target_angle_y))

                # 控制舵机
                servo_x.angle(int(target_angle_x))
                servo_y.angle(int(target_angle_y))

                cv2.putText(img_raw, "Outside 1cm - Servo RUN", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)

            # 显示舵机角度
            cv2.putText(img_raw, f"X: {servo_x.current_angle:.1f}", (10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(img_raw, f"Y: {servo_y.current_angle:.1f}", (10, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        else:
            # 未检测到目标时重置控制项
            integral_x = 0
            integral_y = 0
            derivative_x = 0
            derivative_y = 0
            cv2.putText(img_raw, "No target", (10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 显示图像
        img_show = image.cv2image(img_raw, copy=False)
        disp.show(img_show)

except Exception as e:
    print(f"运行错误: {e}")
finally:
    # 清理资源
    try:
        del servo_x
        del servo_y
    except:
        pass
    print("程序已退出")