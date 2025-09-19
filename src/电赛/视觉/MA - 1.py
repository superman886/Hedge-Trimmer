from maix import image, camera, display, app
import cv2

# 实例化摄像头和显示对象
cam  = camera.Camera(320, 240) 
disp = display.Display()

# 闭运算卷积核
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))

while not app.need_exit():
    img = cam.read()
    img_raw = image.image2cv(img, copy=False) 

    # 转灰度
    img = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY) 
    # 双边滤波
    img = cv2.bilateralFilter(img, 9, 10, 10)
    # 闭运算
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel) 
    # 边缘检测
    edged = cv2.Canny(img, 50, 150) 

    # 找轮廓
    contours, _ = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        # 计算轮廓周长用于多边形逼近
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # 只处理四边形（长方形/矩形）
        if len(approx) == 4:
            # 计算轮廓面积并过滤小面积轮廓
            area = cv2.contourArea(approx)
            if area < 1000:  # 根据实际场景调整阈值
                continue
                
            # 检查是否为凸多边形（矩形是凸的）
            if not cv2.isContourConvex(approx):
                continue
                
            # 计算长宽比
            rect = cv2.minAreaRect(approx)
            (_, _), (width, height), _ = rect
            aspect_ratio = max(width, height) / min(width, height) if min(width, height) > 0 else 0
            
            # 过滤非长方形（长宽比在1.2~5之间）
            if 1.2 <= aspect_ratio <= 5.0:
                # 计算中心坐标
                M = cv2.moments(approx)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # 在原始图像上绘制轮廓
                    cv2.drawContours(img_raw, [approx], 0, (0, 255, 0), 2)
                    
                    # 绘制中心点
                    cv2.circle(img_raw, (cx, cy), 5, (0, 0, 255), -1)
                    
                    # 打印中心坐标（实际使用时可存储或发送）
                    print(f"长方形中心坐标: ({cx}, {cy})")
    
    # 显示处理后的图像
    img_show = image.cv2image(img_raw, copy=False) 
    disp.show(img_show)