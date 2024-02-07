import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile  # Import QoSProfile

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.bridge = CvBridge()

        # Configure QoSProfile
        qos_profile = QoSProfile(depth=10, reliability=QoSProfile.ReliabilityPolicy.RELIABLE)

        # Create publisher with the specified QoS profile
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos_profile)

        self.timer = self.create_timer(0.03, self.timer_callback)  # Adjust frame rate as needed
        self.cap = cv2.VideoCapture('/home/arpan/ros2_ws/src/mot_tracking_package/mot_tracking_package/Data/los_angeles.mp4')
        self.get_logger().info("Video Publisher has been started.")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)
        else:
            self.cap.release()
            self.get_logger().info("Video ended or failed to load. Shutting down.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

