import airsim
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class AirSimImagePublisher(Node):
    def __init__(self):
        super().__init__('airsim_image_publisher')
        
        # Declare parameters
        self.declare_parameter('image_topic', 'airsim/image')
        self.declare_parameter('camera_id', '0')
        self.declare_parameter('frequency', 10.0)
        
        # Get parameters
        topic_name = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_id = self.get_parameter('camera_id').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().double_value
        
        # Create publisher
        self.image_publisher = self.create_publisher(Image, topic_name, 10)
        
        # Create timer for periodic image publishing
        self.timer = self.create_timer(1.0/frequency, self.timer_callback)
        
        self.client = airsim.MultirotorClient(ip='host.docker.internal')
        self.client.confirmConnection()
        self.bridge = CvBridge()
        
        self.get_logger().info(f'Publishing AirSim camera images to {topic_name} at {frequency} Hz')
    
    def timer_callback(self):
        try:
            # Get image from AirSim
            response = self.client.simGetImages([airsim.ImageRequest(
                camera_name=self.camera_id,
                image_type=airsim.ImageType.Scene,
                pixels_as_float=False,
                compress=False
            )])[0]
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 
            img_rgb = img1d.reshape(response.height, response.width, 3)
            
            # Convert to ROS Image message
            ros_img = self.bridge.cv2_to_imgmsg(img_rgb, "rgb8")
            ros_img.header.stamp = self.get_clock().now().to_msg()
            ros_img.header.frame_id = "camera_frame"
            
            # Publish image
            self.image_publisher.publish(ros_img)
            
        except Exception as e:
            self.get_logger().error(f'Error getting image from AirSim: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = AirSimImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

