import cv2
import time
import numpy as np
import onnxruntime
import rospy 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rm_interfaces.msg import Robots
from rm_interfaces.msg import Robot
import threading

class omni_detector:
    def __init__(self):
        rospy.loginfo('start detector')
        self.bridge=CvBridge()
        self.debug=True
        # 加载label names
        self.robots1_pub = rospy.Publisher('/robots1',Robots,queue_size=2)
        self.robots2_pub = rospy.Publisher('/robots2',Robots,queue_size=2)
        self.robots3_pub = rospy.Publisher('/robots3',Robots,queue_size=2)
        self.robots4_pub = rospy.Publisher('/robots4',Robots,queue_size=2)
        self.names = []
        self.input_width=352
        self.input_height=352
        self.img1_sub = rospy.Subscriber("/raw_img",Image,self.img1_callback,queue_size=1)
        self.img2_sub = rospy.Subscriber("/raw_img2",Image,self.img2_callback,queue_size=1)
        self.img3_sub = rospy.Subscriber("/raw_img3",Image,self.img3_callback,queue_size=1)
        self.img4_sub = rospy.Subscriber("/raw_img4",Image,self.img4_callback,queue_size=1)
        self.session = onnxruntime.InferenceSession('/usr/lee/Downloads/aqs-dwr2023-sentry/src/rm_omni/scripts/model/FastestDetcolor.onnx')
        with open("/usr/lee/Downloads/aqs-dwr2023-sentry/src/rm_omni/scripts/model/coco.names", 'r') as f:
            for line in f.readlines():
                self.names.append(line.strip())  

# sigmoid函数
    def sigmoid(self,x):
        return 1. / (1 + np.exp(-x))

    # tanh函数
    def tanh(self, x):
        return 2. / (1 + np.exp(-2 * x)) - 1

    # 数据预处理
    def preprocess(self, src_img, size):
        output = cv2.resize(src_img,(size[0], size[1]),interpolation=cv2.INTER_AREA)
        output = output.transpose(2,0,1)
        output = output.reshape((1, 3, size[1], size[0])) / 255

        return output.astype('float32')

    # nms算法
    def nms(self, dets, thresh=0.45):
        # dets:N*M,N是bbox的个数，M的前4位是对应的（x1,y1,x2,y2），第5位是对应的分数
        # #thresh:0.3,0.5....
        x1 = dets[:, 0]
        y1 = dets[:, 1]
        x2 = dets[:, 2]
        y2 = dets[:, 3]
        scores = dets[:, 4]
        areas = (x2 - x1 + 1) * (y2 - y1 + 1)  # 求每个bbox的面积
        order = scores.argsort()[::-1]  # 对分数进行倒排序
        keep = []  # 用来保存最后留下来的bboxx下标

        while order.size > 0:
            i = order[0]  # 无条件保留每次迭代中置信度最高的bbox
            keep.append(i)

            # 计算置信度最高的bbox和其他剩下bbox之间的交叉区域
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            # 计算置信度高的bbox和其他剩下bbox之间交叉区域的面积
            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h

            # 求交叉区域的面积占两者（置信度高的bbox和其他bbox）面积和的必烈
            ovr = inter / (areas[i] + areas[order[1:]] - inter)

            # 保留ovr小于thresh的bbox，进入下一次迭代。
            inds = np.where(ovr <= thresh)[0]

            # 因为ovr中的索引不包括order[0]所以要向后移动一位
            order = order[inds + 1]
        
        output = []
        for i in keep:
            output.append(dets[i].tolist())

        return output

    # 人脸检测
    def detection(self,session, img, input_width, input_height, thresh):
        pred = []

        # 输入图像的原始宽高
        H, W, _ = img.shape

        # 数据预处理: resize, 1/255
        data = self.preprocess(img, [input_width, input_height])

        # 模型推理
        input_name = session.get_inputs()[0].name
        feature_map = session.run([], {input_name: data})[0][0]

        # 输出特征图转置: CHW, HWC
        feature_map = feature_map.transpose(1, 2, 0)
        # 输出特征图的宽高
        feature_map_height = feature_map.shape[0]
        feature_map_width = feature_map.shape[1]

        # 特征图后处理
        for h in range(feature_map_height):
            for w in range(feature_map_width):
                data = feature_map[h][w]

                # 解析检测框置信度
                obj_score, cls_score = data[0], data[5:].max()
                score = (obj_score ** 0.6) * (cls_score ** 0.4)

                # 阈值筛选
                if score > thresh:
                    # 检测框类别
                    cls_index = np.argmax(data[5:])
                    # 检测框中心点偏移
                    x_offset, y_offset = self.tanh(data[1]), self.tanh(data[2])
                    # 检测框归一化后的宽高
                    box_width, box_height = self.sigmoid(data[3]), self.sigmoid(data[4])
                    # 检测框归一化后中心点
                    box_cx = (w + x_offset) / feature_map_width
                    box_cy = (h + y_offset) / feature_map_height
                    
                    # cx,cy,w,h => x1, y1, x2, y2
                    x1, y1 = box_cx - 0.5 * box_width, box_cy - 0.5 * box_height
                    x2, y2 = box_cx + 0.5 * box_width, box_cy + 0.5 * box_height
                    x1, y1, x2, y2 = int(x1 * W), int(y1 * H), int(x2 * W), int(y2 * H)

                    pred.append([x1, y1, x2, y2, score, cls_index])
        return self.nms(np.array(pred))


    def img1_callback(self,msg):
        img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        new_img = cv2.resize(img,(400,300), interpolation=cv2.INTER_AREA)
        try:
            bboxes = self.detection(self.session, new_img, self.input_width, self.input_height, 0.65)
        except:
            pass
        else:
            robots1=Robots()
            for b in bboxes:
                robot=Robot()
                # print(b)
                obj_score, cls_index = b[4], int(b[5])
                x1, y1, x2, y2 = int(b[0]), int(b[1]), int(b[2]), int(b[3])
                robot.tl.x=x1
                robot.tl.y=y1
                robot.br.x=x2
                robot.br.y=y2
                robot.id=cls_index
                robots1.All_robots.append(robot)
                # rospy.loginfo(robot.tl.y,robot.br.y)
                #绘制检测框
                # print(robot.tl.y)
                # print(robot.br.y)
                cv2.rectangle(new_img, (x1,y1), (x2, y2), (255, 255, 0), 2)
                cv2.putText(new_img, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)
                cv2.putText(new_img, self.names[cls_index], (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)
            cv2.imshow("result1", new_img)
            cv2.waitKey(10)
            robots1.header = msg.header
            self.robots1_pub.publish(robots1)

    def img2_callback(self,msg):
        img = self.bridge.imgmsg_to_cv2(msg,"rgb8")
        new_img = cv2.resize(img,(400,300), interpolation=cv2.INTER_AREA)
        try:
            bboxes = self.detection(self.session, new_img, self.input_width, self.input_height, 0.65)
        except:
            pass
        else:
            robots2=Robots()
            robot=Robot()
            for b in bboxes:
                obj_score, cls_index = b[4], int(b[5])
                x1, y1, x2, y2 = int(b[0]), int(b[1]), int(b[2]), int(b[3])
                robot.tl.x=x1
                robot.tl.y=y2
                robot.br.x=x2
                robot.br.y=y2
                robot.id=cls_index
                robots2.All_robots.append(robot)
                cv2.waitKey(10)
                #绘制检测框
                cv2.rectangle(new_img, (x1,y1), (x2, y2), (255, 255, 0), 2)
                cv2.putText(new_img, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)
                cv2.putText(new_img, self.names[cls_index], (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)
            cv2.imshow("result2", new_img)
            cv2.waitKey(10)
            robots2.header = msg.header
            self.robots2_pub.publish(robots2)

    def img3_callback(self,msg):
        img = self.bridge.imgmsg_to_cv2(msg,"rgb8")
        new_img = cv2.resize(img,(352,352), interpolation=cv2.INTER_AREA)
        try:
            bboxes = self.detection(self.session, new_img, self.input_width, self.input_height, 0.65)
        except:
            pass
        else:
            robots3=Robots()
            robot=Robot()
            for b in bboxes:
                obj_score, cls_index = b[4], int(b[5])
                x1, y1, x2, y2 = int(b[0]), int(b[1]), int(b[2]), int(b[3])
                robot.tl.x=x1
                robot.tl.y=y2
                robot.br.x=x2
                robot.br.y=y2
                robot.id=cls_index
                robots3.All_robots.append(robot)
                cv2.waitKey(10)
                #绘制检测框
                cv2.rectangle(new_img, (x1,y1), (x2, y2), (255, 255, 0), 2)
                cv2.putText(new_img, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)
                cv2.putText(new_img, self.names[cls_index], (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)
            # cv2.imshow("result", new_img)
            # cv2.waitKey(10)
            robots3.header = msg.header
            self.robots2_pub.publish(robots3)

    def img4_callback(self,msg):
        img = self.bridge.imgmsg_to_cv2(msg,"rgb8")
        new_img = cv2.resize(img,(352,352), interpolation=cv2.INTER_AREA)
        try:
            bboxes = self.detection(self.session, new_img, self.input_width, self.input_height, 0.65)
        except:
            pass
        else:
            robots4=Robots()
            robot=Robot()
            for b in bboxes:
                obj_score, cls_index = b[4], int(b[5])
                x1, y1, x2, y2 = int(b[0]), int(b[1]), int(b[2]), int(b[3])
                robot.tl.x=x1
                robot.tl.y=y2
                robot.br.x=x2
                robot.br.y=y2
                robot.id=cls_index
                robots4.All_robots.append(robot)
                cv2.waitKey(10)
                #绘制检测框
                cv2.rectangle(new_img, (x1,y1), (x2, y2), (255, 255, 0), 2)
                cv2.putText(new_img, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)
                cv2.putText(new_img, self.names[cls_index], (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)
            # cv2.imshow("result", new_img)
            # cv2.waitKey(10)
            robots4.header = msg.header
            self.robots4_pub.publish(robots4)
    


def main():
    rospy.init_node("omni_detector", anonymous=True)
    rospy.loginfo('start node')
    detector = omni_detector()
    while(not rospy.is_shutdown()):
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


