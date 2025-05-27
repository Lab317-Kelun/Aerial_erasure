#!usr/bin/env python3
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from cv_task.msg import cv_data

count1 = 0 #计时器
count2 = 0


#调整曝光度函数
def gamma_trans(img,gamma):#gamma函数处理
    gamma_table=[np.power(x/255.0,gamma)*255.0 for x in range(256)]#建立映射表
    gamma_table=np.round(np.array(gamma_table)).astype(np.uint8)#颜色值为整数
    return cv2.LUT(img,gamma_table)#图片颜色查表。另外可以根据光强（颜色）均匀化原则设计自适应算法。
def nothing(x):
    pass
cv2.namedWindow("demo",0)#将显示窗口的大小适应于显示器的分辨率
cv2.createTrackbar('Value of Gamma','demo',700,1000,nothing)#使用滑动条动态调节参数gamma


if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('red_led_detection')

    # 创建发布者
    pub = rospy.Publisher('cv_task',cv_data, queue_size=10)
    cv_Data = cv_data()
    # 打开摄像头
    cap = cv2.VideoCapture(0)   

    while True:
        ret, frame = cap.read()
        #print({frame.shape})    #480, 640
        
        if not ret:
            break
        value_of_gamma=cv2.getTrackbarPos('Value of Gamma','demo')#gamma取值
        value_of_gamma=value_of_gamma*0.01#压缩gamma范围，以进行精细调整
        image_gamma_correct=gamma_trans(frame,7)#2.5为gamma函数的指数值，大于1曝光度下降，大于0小于1曝光度增强
        cv2.imshow("demo",image_gamma_correct)
        # 将图像从BGR转换为HSV
        hsv = cv2.cvtColor(image_gamma_correct, cv2.COLOR_BGR2HSV)
        
  

        # 定义红色范围                                                                   #
        # lower_red = np.array([0, 100, 100])
        lower_red = np.array([156, 100, 100])    # 156,43,46    0 43 46
        # upper_red = np.array([10, 255, 255])
        upper_red = np.array([180, 255, 255])  #180 255 255 


        # 创建红色掩模
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # 寻找红色LED灯的轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  #

        if contours:
            # 获取最大轮廓
            max_contour = max(contours, key=cv2.contourArea)
            
            # 计算最大轮廓的外接正方形
            x, y, w, h = cv2.boundingRect(max_contour)
            
            # 绘制外接正方形
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)  # 红色线条

            # 计算最大轮廓的中心坐标
            M = cv2.moments(max_contour) #M为轮廓的矩
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"]) #水平
                cy = int(M["m01"] / M["m00"]) #垂直
                # cz = int(M["m01"] / M["m00"]) #垂直
                
                # 计算相对于视野中心的x和y轴像素值
                #x_offset = cx - (frame.shape[1] // 2)
                #y_offset = cy - (frame.shape[0] // 2)
                cy = 480 - cy
                # cz = 480 - cz
                # 将坐标值发布到ROS节点
                if cx and cy :
                # if cx and cz :
                    count1 = count1 + 1
                    if count1 > 10:
                        count2 = 0
                        cv_Data.cv_control_flag = 1
                        cv_Data.cv_x = cx
                        cv_Data.cv_y = cy                    
                        # cv_Data.cv_z = cz
                        
                        rospy.loginfo(1)
                        pub.publish(cv_Data)
            else:
                rospy.loginfo("Error")
                cv_Data.cv_control_flag = 0
                pub.publish(cv_Data)

        else:
            count2 = count2 + 1
            if count2 > 10:
                count1 = 0
                rospy.loginfo(0)
                cv_Data.cv_control_flag = 0
                pub.publish(cv_Data)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()



