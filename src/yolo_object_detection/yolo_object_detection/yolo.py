#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import cv2
import torch
from cv_bridge import CvBridge
from ultralytics import YOLO
from torch.backends import cudnn
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D, Pose2D, Point2D

class YoloDetector(Node):
    
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Declare all parameters directly - will be set via launch file
        self.declare_parameter('yolo_model_name', 'yolov8n.pt')
        self.declare_parameter('input_image_topic', '/image')
        self.declare_parameter('inference_topic', '/yolo/inference')
        self.declare_parameter('annotated_frame_topic', '/yolo/annotated_frame')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.4)
        
        # Get parameters from the parameter server (loaded from YAML)
        self.yolo_model_name = self.get_parameter('yolo_model_name').value
        self.input_image_topic = self.get_parameter('input_image_topic').value
        self.inference_topic = self.get_parameter('inference_topic').value
        self.annotated_frame_topic = self.get_parameter('annotated_frame_topic').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value

        # Initialize YOLO right away
        self.get_logger().info(f"Starting YOLO detector...")
        self.get_logger().info(f"Parameters loaded:")
        self.get_logger().info(f"  Model: {self.yolo_model_name}")
        self.get_logger().info(f"  Input topic: {self.input_image_topic}")
        self.get_logger().info(f"  Inference topic: {self.inference_topic}")
        self.get_logger().info(f"  Annotated frame topic: {self.annotated_frame_topic}")
        self.get_logger().info(f"  Confidence threshold: {self.confidence_threshold}")
        self.get_logger().info(f"  IoU threshold: {self.iou_threshold}")

        try:
            self.model = YOLO(self.yolo_model_name)
            self.get_logger().info(f"Using {self.yolo_model_name} for inference")
            self.is_yolo_ready = True
        except Exception as e:
            self.get_logger().error(f"Cannot load {self.yolo_model_name}: {str(e)}")
            self.is_yolo_ready = False
        # cudnn.benchmark = True

        self.bridge = CvBridge()
        self.annotated_frame_publisher = self.create_publisher(Image, self.annotated_frame_topic, 10)
        self.inference_publisher = self.create_publisher(Detection2DArray, self.inference_topic, 10)
        self.image_subscriber = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_subscriber_callback,
            10
        )
        
        self.get_logger().info("YOLO detector is ready!")

    def image_subscriber_callback(self, msg):
        if not self.is_yolo_ready:
            return
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return
        
        results = self.model.track(source=frame, conf=self.confidence_threshold, iou=self.iou_threshold, verbose=False)[0]
        self.publish_box_inference(results)

        # Publish the annotated frame
        try:
            annotated_frame = results.plot()
   
            # Draw a red point at the center of the frame and yellow points at the center of detected objects
            frame_center = (annotated_frame.shape[1] // 2, annotated_frame.shape[0] // 2)
            cv2.drawMarker(annotated_frame, frame_center, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=1)
            for xywh in results.boxes.xywhn:
                box_center = (int(xywh[0] * annotated_frame.shape[1]), int(xywh[1] * annotated_frame.shape[0]))
                cv2.drawMarker(annotated_frame, box_center, (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=1)
    
            # Publish the annotated frame
            self.annotated_frame_publisher.publish(self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8"))
        except Exception as e:
            self.get_logger().error(f"Error processing annotated frame: {str(e)}")

    def publish_box_inference(self, results):
        boxes = results.boxes
        xywhn = boxes.xywhn.cpu().numpy()     # A 2D tensor of shape (N, 4) where N is the number of detected objects
        conf = boxes.conf.cpu().numpy()       # A 1D tensor of shape (N,) containing the confidence scores of the detected objects
        cls = boxes.cls.cpu().numpy()         # A 1D tensor of shape (N,) containing the class labels of the detected objects

        conf = conf.reshape(-1, 1)
        cls = cls.reshape(-1, 1)
        
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "yolo_inference"

        for i in range(xywhn.shape[0]):
            detection = Detection2D()
            
            # Corrected: Use Pose2D for the center as per the BoundingBox2D message definition
            center = Pose2D()
            center.position.x = float(xywhn[i][0])
            center.position.y = float(xywhn[i][1])
            center.theta = 0.0  # Pose2D includes theta which Point2D doesn't have
            
            detection.bbox = BoundingBox2D()
            detection.bbox.center = center
            detection.bbox.size_x = float(xywhn[i][2])
            detection.bbox.size_y = float(xywhn[i][3])

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(int(cls[i]))
            hypothesis.hypothesis.score = float(conf[i])
            detection.results.append(hypothesis)

            msg.detections.append(detection)

        self.inference_publisher.publish(msg)   

    def cleanup(self):
        self.get_logger().info(f"Shutting down {self.get_name()}")
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_detector.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()