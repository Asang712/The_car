#ros lib
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
#common lib
import os
import cv2 as cv
import numpy as np

def ManyImgs(scale, imgarray):
    rows = len(imgarray)  # 元组或者列表的长度 Length of tuple or list
    cols = len(imgarray[0])  # 如果imgarray是列表，返回列表里第一幅图像的通道数，如果是元组，返回元组里包含的第一个列表的长度
    rowsAvailable = isinstance(imgarray[0], list)
    # 第一张图片的宽高
    # The width and height of the first picture
    width = imgarray[0][0].shape[1]
    height = imgarray[0][0].shape[0]
    # print("width=", width, "height=", height)
    # 如果传入的是一个元组
    # If the incoming is a tuple
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                # 遍历元组，如果是第一幅图像，不做变换
                # Traverse the tuple, if it is the first image, do not transform
                if imgarray[x][y].shape[:2] == imgarray[0][0].shape[:2]:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (0, 0), None, scale, scale)
                # 将其他矩阵变换为与第一幅图像相同大小，缩放比例为scale
                # Transform other matrices to the same size as the first image, and the zoom ratio is scale
                else:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (imgarray[0][0].shape[1], imgarray[0][0].shape[0]), None,
                                               scale, scale)
                # 如果图像是灰度图，将其转换成彩色显示
                # If the image is grayscale, convert it to color display
                if len(imgarray[x][y].shape) == 2:
                    imgarray[x][y] = cv.cvtColor(imgarray[x][y], cv.COLOR_GRAY2BGR)
        # 创建一个空白画布，与第一张图片大小相同
        # Create a blank canvas, the same size as the first picture
        imgBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imgBlank] * rows  # 与第一张图片大小相同，与元组包含列表数相同的水平空白图像
        # The same size as the first picture, and the same number of horizontal blank images as the tuple contains the list
        for x in range(0, rows):
            # 将元组里第x个列表水平排列
            # Arrange the x-th list in the tuple horizontally
            hor[x] = np.hstack(imgarray[x])
        ver = np.vstack(hor)  # 将不同列表垂直拼接 Concatenate different lists vertically
    # 如果传入的是一个列表 If the incoming is a list
    else:
        # 变换操作，与前面相同
        # Transformation operation, same as before
        for x in range(0, rows):
            if imgarray[x].shape[:2] == imgarray[0].shape[:2]:
                imgarray[x] = cv.resize(imgarray[x], (0, 0), None, scale, scale)
            else:
                imgarray[x] = cv.resize(imgarray[x], (imgarray[0].shape[1], imgarray[0].shape[0]), None, scale, scale)
            if len(imgarray[x].shape) == 2:
                imgarray[x] = cv.cvtColor(imgarray[x], cv.COLOR_GRAY2BGR)
        # 将列表水平排列
        # Arrange the list horizontally
        hor = np.hstack(imgarray)
        ver = hor
    return ver
class color_follow:
    def __init__(self):
        '''
        初始化一些参数
	    Initialize some parameters
        '''
        self.binary = None
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0

    def line_follow(self, rgb_img, hsv_msg):
        height, width = rgb_img.shape[:2]
        img = rgb_img.copy()
        img[0:int(height / 2), 0:width] = 0
        # 将图像转换为HSV。
        # Convert the image to HSV.
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower = np.array(hsv_msg[0], dtype="uint8")
        upper = np.array(hsv_msg[1], dtype="uint8")
        # 根据特定颜色范围创建mask
        # Create a mask based on a specific color range
        mask = cv.inRange(hsv_img, lower, upper)
        color_mask = cv.bitwise_and(hsv_img, hsv_img, mask=mask)
        # 将图像转为灰度图
        # Convert the image to grayscale
        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)
        # 获取不同形状的结构元素
        # Get structure elements of different shapes
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # 形态学闭操作
        # Morphological closed operation
        gray_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        # 图像二值化操作
        # Image binarization operation
        ret, binary = cv.threshold(gray_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标)
        # Get the set of contour points (coordinates)
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(find_contours) == 3: contours = find_contours[1]
        else: contours = find_contours[0]
        if len(contours) != 0:
            areas = []
            for c in range(len(contours)): areas.append(cv.contourArea(contours[c]))
            max_id = areas.index(max(areas))
            max_rect = cv.minAreaRect(contours[max_id])
            max_box = cv.boxPoints(max_rect)
            max_box = np.int0(max_box)
            # 获取盒⼦顶点
            # Get box vertices
            box = cv.boxPoints(max_rect)
            # 转成long类型
            # Convert to long type
            box = np.int0(box)
            # 绘制最小矩形
            # Draw the smallest rectangle
            cv.drawContours(rgb_img, [box], 0, (255, 0, 0), 2)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(max_box)
            # 将检测到的颜色用原形线圈标记出来
            # Mark the detected color with the original shape coil
            self.Center_x = int(color_x)
            self.Center_y = int(color_y)
            self.Center_r = int(color_radius)
            cv.circle(rgb_img, (self.Center_x, self.Center_y), 5, (255, 0, 255), -1)
        else:
            self.Center_x = 0
            self.Center_y = 0
            self.Center_r = 0
        return rgb_img, binary, (self.Center_x, self.Center_y, self.Center_r)

    def Roi_hsv(self, img, Roi):
        '''
        Get the range of HSV in a certain area获取某一区域的HSV的范围
        :param img: 彩色图
        :param Roi:  (x_min, y_min, x_max, y_max)
        Roi=(290,280,350,340)
        :return: The range of images and HSV such as:(0,0,90)(177,40,150) 图像和HSV的范围 例如：(0,0,90)(177,40,150)
        '''
        H = [];S = [];V = []
        # img = cv.resize(img, (640, 480), )
        # 将彩色图转成HSV
        # Convert color image to HSV
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # 画矩形框
        # Draw a rectangular frame
        # cv.rectangle(img, (Roi[0], Roi[1]), (Roi[2], Roi[3]), (0, 255, 0), 2)
        # 依次取出每行每列的H,S,V值放入容器中
        # Take out the H, S, V values of each row and each column in turn and put them into the container
        for i in range(Roi[0], Roi[2]):
            for j in range(Roi[1], Roi[3]):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])
        # 分别计算出H,S,V的最大最小
        # Calculate the maximum and minimum of H, S, and V respectively
        H_min = min(H); H_max = max(H)
        S_min = min(S); S_max = 253
        V_min = min(V); V_max = 255
        # HSV范围调整
        # HSV range adjustment
        if H_max + 5 > 255: H_max = 255
        else: H_max += 5
        if H_min - 5 < 0: H_min = 0
        else: H_min -= 5
        if S_min - 20 < 0: S_min = 0
        else: S_min -= 20
        if V_min - 20 < 0: V_min = 0
        else: V_min -= 20
        lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min) + ')'
        upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max) + ')'
        txt1 = 'Learning ...'
        txt2 = 'OK !!!'
        if S_min < 5 or V_min < 5:
            cv.putText(img, txt1, (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            cv.putText(img, txt2, (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv.putText(img, lowerb, (150, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv.putText(img, upperb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        hsv_range = ((int(H_min), int(S_min), int(V_min)), (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range




class ColorSelector(Node):
    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {str(e)}")

    def __init__(self,name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.selected_count = 1
        self.colors = [()]*5
        self.finish_flag = False
        
        
        self.cv_image = None
        cv.namedWindow('Color Selector')
        cv.setMouseCallback('Color Selector', self.on_mouse) 
        
        self.color_tool = color_follow()
        self.Roi_init = ()
        self.select_flags = False
        self.cols, self.rows = (0, 0), (0, 0)
        self.preview_hsv = None
        # 相机发布端使用 BEST_EFFORT，订阅端必须对齐，否则收不到图像
        camera_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.sub_image = self.create_subscription(Image, "/camera/image_raw", self.image_callback, camera_qos)
        self.declare_parameter('hsv_text_file', "")
        self.hsv_text = self.get_parameter('hsv_text_file').get_parameter_value().string_value
        
    def on_mouse(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.select_flags = True
            self.Mouse_XY = (x, y)
            self.preview_hsv = None
            self.cols = (x, y)
            self.rows = (x, y)
            
        elif event == cv.EVENT_MOUSEMOVE:
            if self.select_flags and self.cv_image is not None:
                img_h, img_w = self.cv_image.shape[:2]
                x = max(0, min(x, img_w-1))
                y = max(0, min(y, img_h-1))
                self.cols = (min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y))
                self.rows = (max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y))
                self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])
        elif event == cv.EVENT_LBUTTONUP:
            self.select_flags = False
            if self.Roi_init and abs(x - self.Mouse_XY[0]) > 2 and abs(y - self.Mouse_XY[1]) > 2:
                try:
                    full_frame = self.cv_image.copy()
                    roi_frame = full_frame[self.Roi_init[1]:self.Roi_init[3], self.Roi_init[0]:self.Roi_init[2]]
                    _, fixed_hsv = self.color_tool.Roi_hsv(roi_frame, (0, 0, roi_frame.shape[1], roi_frame.shape[0]))
                    if fixed_hsv: self.preview_hsv = fixed_hsv
                except Exception as e:
                    self.get_logger().error(f"HSV Error: {str(e)}")

    def process_frame(self, action):
        if self.cv_image is None: return
        
        frame = self.cv_image.copy()
        frame = cv.resize(frame, (640, 480))
        binary = []
        
        if self.select_flags:
            x1, y1 = max(0, self.cols[0]), max(0, self.cols[1])
            x2, y2 = min(frame.shape[1], self.rows[0]), min(frame.shape[0], self.rows[1])
            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
        if self.preview_hsv and not self.select_flags:
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            binary = cv.inRange(hsv, self.preview_hsv[0], self.preview_hsv[1])
            
            hsv_max = f"MAX: HSV({self.preview_hsv[1][0]}, {self.preview_hsv[1][1]}, {self.preview_hsv[1][2]})"
            hsv_min = f"MIN: HSV({self.preview_hsv[0][0]}, {self.preview_hsv[0][1]}, {self.preview_hsv[0][2]})"
            
            (text_w, text_h), _ = cv.getTextSize(hsv_max, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            x_pos = (frame.shape[1] - text_w) // 2
            
            cv.putText(frame, hsv_max, (x_pos, 25), 
                      cv.FONT_HERSHEY_SIMPLEX, 0.5,(255, 0, 0), 1)
            cv.putText(frame, hsv_min, (x_pos, 45), 
                      cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        color_map = {1: "RED", 2: "GREEN", 3: "BLUE", 4: "YELLOW"}
        prefix = color_map.get(self.selected_count, "RGBY")
        status_text = f"{prefix} Color {self.selected_count}/4"
        if self.finish_flag: status_text = "FINISH"
        cv.putText(frame, status_text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        if action == 32:
            if not self.finish_flag and self.preview_hsv and 1 <= self.selected_count <= 4:
                try:
                    with open(self.hsv_text, 'r') as f:
                        lines = f.readlines()
                except FileNotFoundError:
                    lines = ['\n'] * 4

                lines = lines[:4]
                while len(lines) < 4:
                    lines.append('\n')

                new_line = ', '.join(map(str, self.preview_hsv[0] + self.preview_hsv[1])) + '\n'
                lines[self.selected_count - 1] = new_line

                with open(self.hsv_text, 'w') as f:
                    f.writelines(lines)

                self.colors[self.selected_count - 1] = self.preview_hsv
                if self.selected_count < 4:
                    self.selected_count += 1
                else:
                    self.finish_flag = True
                self.preview_hsv = None
                self.Roi_init = ()

        elif action == ord('e') or action == ord('E'):
            self.selected_count += 1
            self.preview_hsv = None
            if self.selected_count > 4:
                self.selected_count = 1

        #status_text = f"RGBY Color {self.selected_count}/4"
        if self.finish_flag:
            status_text = "FINISH"
        cv.putText(frame, status_text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
    
        if binary is not None and len(binary) > 0 and self.preview_hsv is not None:
            display_frame = ManyImgs(1, ([frame, cv.cvtColor(binary, cv.COLOR_GRAY2BGR)]))
        else:
            display_frame = frame
            
        cv.imshow('Color Selector', display_frame)

def main(args=None):
    rclpy.init(args=args)
    selector = ColorSelector("color_selector")
    
    try:
        while rclpy.ok():
            rclpy.spin_once(selector, timeout_sec=0.1)
            action = cv.waitKey(10) & 0xFF
            selector.process_frame(action)
            if action == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv.destroyAllWindows()
        selector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
