#!/usr/bin/python3
import os, sys

from models.experimental import attempt_load
from utils.general import non_max_suppression
from utils.ros import create_detection_msg
from visualizer import draw_detections

from typing import Tuple, Union, List

import torch
import cv2
from torchvision.transforms import ToTensor
import numpy as np
import rospy

from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class Yolov7Publisher:
    def __init__(self, img_topic: str, weights: str, conf_thresh: float,
                 iou_thresh: float, pub_topic: str,
                 device: str,
                 img_size: Union[Tuple[int, int], None] = (640, 640),
                 queue_size: int = 1, visualize: bool = False,
                 class_labels: Union[List, None] = None):
        
        self.img_size = img_size
        self.device = device
        self.class_labels = class_labels
        self.bridge = CvBridge()
        self.tensorize = ToTensor()
        self.image = None
        self.img_check = False

        vis_topic = pub_topic + "visualization" if pub_topic.endswith("/") else \
            pub_topic + "/visualization"
        
        self.visualization_publisher = rospy.Publisher(vis_topic, Image, queue_size=queue_size) if visualize else None
        self.img_subscriber = rospy.Subscriber(img_topic, CompressedImage, self.image_callback)
        self.detection_publisher = rospy.Publisher(pub_topic, Detection2DArray, queue_size=queue_size)

        self.model = YoloV7(
            weights=weights, conf_thresh=conf_thresh, iou_thresh=iou_thresh,
            device=device
        )

        self.frequency = 40
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.frequency), self.timer_callback)

        rospy.loginfo("YOLOv7 initialization complete. Ready to start inference")

    def timer_callback(self, event):
        if self.img_check:
            self.yolo()
            self.img_check = False

    def image_callback(self, img_msg: CompressedImage):

        np_arr = np.frombuffer(img_msg.data, np.uint8)
        np_img_orig = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # handle possible different img formats
        if len(np_img_orig.shape) == 2:
            np_img_orig = np.stack([np_img_orig] * 3, axis=2)

        self.image = np_img_orig.copy()
        self.img_check = True

    def yolo(self):

        h_orig, w_orig, c = self.image.shape

        # automatically resize the image to the next smaller possible size
        w_scaled, h_scaled = self.img_size
        np_img_resized = cv2.resize(self.image, (w_scaled, h_scaled))

        # conversion to torch tensor (copied from original yolov7 repo)
        img = np_img_resized.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = torch.from_numpy(np.ascontiguousarray(img))
        img = img.float()  # uint8 to fp16/32
        img /= 255  # 0 - 255 to 0.0 - 1.
        img = img.to(self.device)

        # inference & rescaling the output to original img size
        detections = self.model.inference(img)
        detections[:, :4] = rescale(
            [h_scaled, w_scaled], detections[:, :4], [h_orig, w_orig])
        detections[:, :4] = detections[:, :4].round()

        # publishing
        detection_msg = create_detection_msg(detections)
        self.detection_publisher.publish(detection_msg)

        # visualizing if required
        if self.visualization_publisher:
            bboxes = [[int(x1), int(y1), int(x2), int(y2)]
                      for x1, y1, x2, y2 in detections[:, :4].tolist()]
            classes = [int(c) for c in detections[:, 5].tolist()]
            vis_img = draw_detections(self.image, bboxes, classes,
                                      self.class_labels)
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding="bgr8")
            self.visualization_publisher.publish(vis_msg)

def parse_classes_file(path):
    classes = []
    with open(path, "r") as f:
        for line in f:
            line = line.replace("\n", "")
            classes.append(line)
    return classes


def rescale(ori_shape: Tuple[int, int], boxes: Union[torch.Tensor, np.ndarray],
            target_shape: Tuple[int, int]):
    xscale = target_shape[1] / ori_shape[1]
    yscale = target_shape[0] / ori_shape[0]

    boxes[:, [0, 2]] *= xscale
    boxes[:, [1, 3]] *= yscale

    return boxes


class YoloV7:
    def __init__(self, weights, conf_thresh: float = 0.5, iou_thresh: float = 0.45,
                 device: str = "cuda"):
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh
        self.device = device
        self.model = attempt_load(weights, map_location=device)
        self.model.eval()

    @torch.no_grad()
    def inference(self, img: torch.Tensor):

        img = img.unsqueeze(0)
        pred_results = self.model(img)[0]
        detections = non_max_suppression(
            pred_results, conf_thres=self.conf_thresh, iou_thres=self.iou_thresh
        )
        if detections:
            detections = detections[0]
        return detections

if __name__ == "__main__":
    rospy.init_node("yolov7_node")

    ns = rospy.get_name() + "/"

    weights_path = "/root/deli/catkin_ws/src/object_detection/model_pt/1010_best.pt"
    classes_path = "/root/deli/catkin_ws/src/object_detection/class_labels/list.txt"
    img_topic = "/image_jpeg/compressed"
    out_topic = "/yolo/results"
    conf_thresh = 0.5
    iou_thresh = 0.45
    queue_size = 1
    img_size = 640
    visualize = False
    device = "cuda"

    # some sanity checks
    if not os.path.isfile(weights_path):
        raise FileExistsError(f"Weights not found ({weights_path}).")
    
    if classes_path: 
        if not os.path.isfile(classes_path):
            raise FileExistsError(f"Classes file not found ({classes_path}).")
        classes = parse_classes_file(classes_path)
    else:
        rospy.loginfo("No class file provided. Class labels will not be visualized.")
        classes = None

    if not ("cuda" in device or "cpu" in device):
        raise ValueError("Check your device.")


    publisher = Yolov7Publisher(
        img_topic=img_topic,
        pub_topic=out_topic,
        weights=weights_path,
        device=device,
        visualize=visualize,
        conf_thresh=conf_thresh,
        iou_thresh=iou_thresh,
        img_size=(img_size, img_size),
        queue_size=queue_size,
        class_labels=classes
    )

    rospy.spin()