#!/usr/bin/env python

import os
import sys
import rospy
import cv2
import copy
import torch
import time
from cv_bridge import CvBridge, CvBridgeError
from detector.detect_utils import *
from detector.trt_model import TrtModel
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from rm_interfaces.msg import Armor
from rm_interfaces.msg import AllTargets

class Detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.classes = ["Dark1", "Dark2", "Dark3", "Dark4", "Dark5", "Dark6", "Dark7",  "Red0", "Red1", "Red2", "Red3", "Red4", "Red5", "Red6", "Red7","Red8", "Blue0", "Blue1", "Blue2", "Blue3", "Blue4", "Blue5", "Blue6", "Blue7", "Blue8"]
        self.img_sub = rospy.Subscriber("/raw_img",Image,self.img_callback,queue_size=1)
        self.targets_pub = rospy.Publisher('/all_targets',AllTargets,queue_size=20)
        self.debug = rospy.get_param("/debug",default=True)
        self.conf_thresh = rospy.get_param("/detector/conf_thresh",default=0.2)
        self.iou_thresh = rospy.get_param("/detector/iou_thresh",default=0.2)
        self.enemy_color = rospy.get_param("/enemy_color")
        trt_path = rospy.get_param("/detector/model_path")
        self.model=TrtModel(trt_path)
        self.start_time = time.time()
    
    def __del__(self):
        # self.model.destroy()  
        self.model.destroy();

    def img_callback(self,msg):
        img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        if self.debug:
            self.start_time = time.time()
        processed_img = self.img_process(orgimg=img, long_side=640, stride_max=32)
        pred=self.model(processed_img.numpy()).reshape([1,25200,40]) # forward
        pred = non_max_suppression_face(torch.from_numpy(pred), conf_thres=self.conf_thresh, iou_thres=self.iou_thresh)
        armors = self.post_process(img, processed_img, pred)
        if armors is None:
            return
        for armor in armors:
            if (self.enemy_color == 0) and (armor.id < 8 or armor.id > 15):
                armors.remove(armor)
            elif (self.enemy_color == 1) and (armor.id < 17):
                armors.remove(armor)
        all_targets = AllTargets()
        all_targets.header = msg.header
        all_targets.armors = armors
        all_targets.offset.x = 240
        all_targets.offset.y = 240
        self.targets_pub.publish(all_targets)



    def img_process(self, orgimg, long_side=640, stride_max=32):
        '''
        图像预处理
        '''
        img0 = copy.deepcopy(orgimg)
        h0, w0 = orgimg.shape[:2]  # orig hw
        r = long_side/ max(h0, w0)  # resize image to img_size
        if r != 1:  # always resize down, only resize up if training with augmentation
            interp = cv2.INTER_AREA if r < 1 else cv2.INTER_LINEAR
            img0 = cv2.resize(img0, (int(w0 * r), int(h0 * r)), interpolation=interp)

        imgsz = check_img_size(long_side, s=stride_max)  # check img_size

        img = letterbox(img0, new_shape=imgsz,auto=False)[0] # auto True最小矩形   False固定尺度
        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1).copy()  # BGR to RGB, to 3x416x416
        img = torch.from_numpy(img)
        img = img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        return img

    def post_process(self, src, processed_img, pred):
        '''
        后处理
        '''
        armors = list()
        # Process detections
        for i, det in enumerate(pred):  # detections per image
            gn = torch.tensor(src.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            gn_lks = torch.tensor(src.shape)[[1, 0, 1, 0, 1, 0, 1, 0, 1, 0]]  # normalization gain landmarks
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(processed_img.shape[2:], det[:, :4], src.shape).round()
          
                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class

                det[:, 5:15] = scale_coords_landmarks(processed_img.shape[2:], det[:, 5:15], src.shape).round()

                for j in range(det.size()[0]):

                    conf = det[j, 4].cpu().numpy()
                    landmarks = (det[j, 5:15].view(1, 10)).view(-1).tolist()
                    class_num = det[j, 15:].cpu().numpy()
                    best_cls = 0
                    if len(class_num) > 0:
                        best_cls = (int)(class_num[np.argmax(class_num)])
                    else:
                        if self.debug:
                            img = src
                            dt = time.time() - self.start_time
                            cv2.putText(img,"fps:{}".format(1.0/dt),(5,30),cv2.FONT_HERSHEY_SIMPLEX,1,[0,255,0])
                            cv2.imshow("results",img)
                            cv2.waitKey(10)
                            return None

                    armor  = Armor()
                    armor.id = best_cls
                    armor.tl.x = landmarks[0]
                    armor.tl.y = landmarks[1]
                    armor.bl.x = landmarks[2]
                    armor.bl.y = landmarks[3]
                    armor.br.x = landmarks[4]
                    armor.br.y = landmarks[5]
                    armor.tr.x = landmarks[6]
                    armor.tr.y = landmarks[7]
                    armors.append(armor)
                    if self.debug:
                        pt_xyxy = (det[j, :4].view(1, 4)).view(-1).tolist()
                        img = show_results(src,pt_xyxy,self.conf_thresh,landmarks,best_cls)
                        dt = time.time() - self.start_time
                        cv2.putText(img,"fps:{}".format(1.0/dt),(5,30),cv2.FONT_HERSHEY_SIMPLEX,1,[0,255,0])
                        cv2.imshow("results",img)
                        cv2.waitKey(1)
        return armors

def main():
    rospy.init_node("detector", anonymous=True)
    detector = Detector()
    while(not rospy.is_shutdown()):
      rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
