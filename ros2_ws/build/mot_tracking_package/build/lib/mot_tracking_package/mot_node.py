import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile

class ObjectDetection:
    def __init__(self, weights_path, cfg_path):
        print("Loading Object Detection")
        print("Running opencv dnn with YOLOv4")
        self.nmsThreshold = 0.4
        self.confThreshold = 0.5
        self.image_size = 608

        net = cv2.dnn.readNet(weights_path, cfg_path)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        self.model = cv2.dnn_DetectionModel(net)
        self.model.setInputParams(size=(self.image_size, self.image_size), scale=1/255)

        self.classes = []
        self.load_class_names(cfg_path.replace("yolov4.cfg", "classes.txt"))

    def load_class_names(self, classes_path):
        with open(classes_path, "r") as file_object:
            self.classes = [class_name.strip() for class_name in file_object.readlines()]

    def detect(self, frame):
        return self.model.detect(frame, nmsThreshold=self.nmsThreshold, confThreshold=self.confThreshold)

class MOTNode(Node):
    def __init__(self):
        super().__init__('mot_node')
        self.bridge = CvBridge()
        weights_path = "/home/arpan/ros2_ws/src/mot_tracking_package/mot_tracking_package/dnn_model/yolov4.weights"
        cfg_path = "/home/arpan/ros2_ws/src/mot_tracking_package/mot_tracking_package/dnn_model/yolov4.cfg"
        self.od = ObjectDetection(weights_path, cfg_path)
        self.publisher = self.create_publisher(String, '/tracking_data', 10)

        # Configure QoSProfile
        qos_profile = QoSProfile(depth=10, reliability=QoSProfile.ReliabilityPolicy.RELIABLE)

        # Create subscriber with the specified QoS profile
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile)

        self.center_points_prev_frame = []
        self.tracking_objects = {}
        self.track_id = 0


    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        class_ids, scores, boxes = self.od.detect(frame)
        print(f"Detections: {len(boxes)}")
        center_points_cur_frame = [(int((x + w / 2)), int((y + h / 2))) for x, y, w, h in boxes]

        if not self.center_points_prev_frame:
            for i, pt in enumerate(center_points_cur_frame):
                self.tracking_objects[self.track_id] = pt
                self.track_id += 1
        else:
            pass
       
        tracking_info = String()
        tracking_info.data = f"Tracking {len(self.tracking_objects)} objects"
        self.publisher.publish(tracking_info)
        self.center_points_prev_frame = center_points_cur_frame.copy()

def main(args=None):
    rclpy.init(args=args)
    node = MOTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

