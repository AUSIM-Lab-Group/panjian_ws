#!/usr/bin/env python3
"""
YOLOv8n Semantic Detection Node
Subscribes to RGB image, runs YOLOv8n inference, publishes Detection2DArray.
"""
import rospy
import numpy as np
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    rospy.logwarn("ultralytics not installed. YOLO node will publish empty detections.")


# COCO class ID → semantic class mapping
COCO_TO_SEMANTIC = {
    0: "pedestrian",   # person (default; child determined by bbox height)
    1: "cyclist",      # bicycle
    2: "vehicle",      # car
    5: "vehicle",      # bus
    7: "vehicle",      # truck
    24: "box",         # backpack
    28: "box",         # suitcase
}

# If person bbox height < this fraction of image height, classify as child
CHILD_HEIGHT_RATIO = 0.35


class YoloNode:
    def __init__(self):
        rospy.init_node("yolo_node")

        self.bridge = CvBridge()
        self.model = None

        # Parameters
        self.model_path = rospy.get_param("~model_path", "yolov8n.pt")
        self.device = rospy.get_param("~device", "cuda:0")
        self.conf_thresh = rospy.get_param("~confidence_threshold", 0.5)
        image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")

        # Load model
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(self.model_path)
                rospy.loginfo(f"YOLOv8 model loaded from {self.model_path} on {self.device}")
            except Exception as e:
                rospy.logerr(f"Failed to load YOLO model: {e}")
        else:
            rospy.logwarn("Running in dummy mode (no ultralytics)")

        # Publishers / Subscribers
        self.pub_det = rospy.Publisher("/yolo/detections", Detection2DArray, queue_size=1)
        self.pub_vis = rospy.Publisher("/yolo/image_vis", Image, queue_size=1)
        self.sub_img = rospy.Subscriber(image_topic, Image, self.image_cb, queue_size=1, buff_size=2**24)

        rospy.loginfo(f"YOLO node ready. Subscribing to {image_topic}")

    def image_cb(self, msg):
        """Process incoming image and publish detections."""
        det_array = Detection2DArray()
        det_array.header = msg.header

        if self.model is None:
            self.pub_det.publish(det_array)
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"cv_bridge error: {e}")
            self.pub_det.publish(det_array)
            return

        img_h, img_w = img.shape[:2]

        # Run inference
        results = self.model(img, conf=self.conf_thresh, device=self.device, verbose=False)

        if len(results) == 0 or results[0].boxes is None:
            self.pub_det.publish(det_array)
            return

        boxes = results[0].boxes
        for i in range(len(boxes)):
            cls_id = int(boxes.cls[i].item())
            conf = float(boxes.conf[i].item())
            x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy()

            # Filter to target classes
            if cls_id not in COCO_TO_SEMANTIC:
                continue

            semantic_class = COCO_TO_SEMANTIC[cls_id]

            # Person → child heuristic (small bbox)
            if cls_id == 0:
                bbox_h = y2 - y1
                if bbox_h / img_h < CHILD_HEIGHT_RATIO:
                    semantic_class = "child"

            # Build Detection2D
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.x = (x1 + x2) / 2.0
            det.bbox.center.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1

            hyp = ObjectHypothesisWithPose()
            hyp.id = cls_id
            hyp.score = conf
            # Store semantic class in the hypothesis id field as string workaround
            # Actual semantic class passed via source_img.header.frame_id trick or custom field
            det.results.append(hyp)

            # Use source_img encoding field to pass semantic class (hack for vision_msgs limitation)
            det.source_img.encoding = semantic_class

            det_array.detections.append(det)

        self.pub_det.publish(det_array)

        # Publish visualization image with bounding boxes
        if self.pub_vis.get_num_connections() > 0:
            import cv2
            vis_img = img.copy()
            for det in det_array.detections:
                cls_name = det.source_img.encoding
                cx, cy = det.bbox.center.x, det.bbox.center.y
                w, h = det.bbox.size_x, det.bbox.size_y
                x1_d, y1_d = int(cx - w/2), int(cy - h/2)
                x2_d, y2_d = int(cx + w/2), int(cy + h/2)
                score = det.results[0].score if det.results else 0

                # Color by class
                colors = {"pedestrian": (0,255,0), "child": (0,255,255),
                          "cyclist": (255,255,0), "vehicle": (255,0,0),
                          "box": (128,128,128), "unknown": (200,200,200)}
                color = colors.get(cls_name, (255,255,255))

                cv2.rectangle(vis_img, (x1_d, y1_d), (x2_d, y2_d), color, 2)
                label = f"{cls_name} {score:.2f}"
                cv2.putText(vis_img, label, (x1_d, y1_d - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding="bgr8")
            vis_msg.header = msg.header
            self.pub_vis.publish(vis_msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = YoloNode()
    node.run()
