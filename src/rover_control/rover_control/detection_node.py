#!/usr/bin/env python3
# detection_node: runs YOLO on the Pi camera, publishes ALL detections
# per type as Float32MultiArray: [image_width, cx1, cx2, ...]
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
import cv2
from ultralytics import YOLO
from picamera2 import Picamera2

IMG_W = 640
IMG_H = 480

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        self.human_model = YOLO('/home/minions/yolov8n.pt')
        self.fire_model = YOLO('/home/minions/best.pt')

        self.picam2 = Picamera2()
        self.picam2.preview_configuration.main.size = (IMG_W, IMG_H)
        self.picam2.preview_configuration.main.format = "RGB888"
        self.picam2.configure("preview")
        self.picam2.start()

        # publishers: each carries [img_width, cx1, cx2, ...]
        self.fire_pub = self.create_publisher(Float32MultiArray, '/det/fire', 10)
        self.smoke_pub = self.create_publisher(Float32MultiArray, '/det/smoke', 10)
        self.human_pub = self.create_publisher(Float32MultiArray, '/det/human', 10)
        self.frame_pub = self.create_publisher(CompressedImage, '/camera/annotated', 10)
        self.create_timer(0.5, self.detect)
        self.get_logger().info('Detection node started (multi-detection)')

    def centers_for(self, results, model, want_name):
        """Return list of bounding-box center-x for detections matching want_name."""
        centers = []
        for box in results[0].boxes:
            cls = int(box.cls[0])
            name = model.names[cls]
            if name == want_name:
                x1, _, x2, _ = box.xyxy[0]
                cx = float((x1 + x2) / 2.0)
                centers.append(cx)
        return centers

    def detect(self):
        frame = self.picam2.capture_array()

        # humans (class 0 in yolov8n = 'person')
        hres = self.human_model(frame, imgsz=320, classes=[0], conf=0.5, verbose=False)
        human_centers = [float((b.xyxy[0][0] + b.xyxy[0][2]) / 2.0)
                         for b in hres[0].boxes]

        # fire/smoke (custom model)
        fres = self.fire_model(frame, imgsz=320, conf=0.4, verbose=False)
        fire_centers = self.centers_for(fres, self.fire_model, 'fire')
        smoke_centers = self.centers_for(fres, self.fire_model, 'smoke')

        # publish [img_width, cx1, cx2, ...]
        self.human_pub.publish(Float32MultiArray(data=[float(IMG_W)] + human_centers))
        self.fire_pub.publish(Float32MultiArray(data=[float(IMG_W)] + fire_centers))
        self.smoke_pub.publish(Float32MultiArray(data=[float(IMG_W)] + smoke_centers))

        if human_centers or fire_centers or smoke_centers:
            self.get_logger().info(
                f'humans={len(human_centers)} fire={len(fire_centers)} smoke={len(smoke_centers)}')

        # draw boxes for the dashboard feed
        annotated = frame.copy()
        for b in hres[0].boxes:
            x1, y1, x2, y2 = map(int, b.xyxy[0])
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, "person", (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        for b in fres[0].boxes:
            cls = int(b.cls[0])
            name = self.fire_model.names[cls]
            x1, y1, x2, y2 = map(int, b.xyxy[0])
            color = (0, 0, 255) if name == 'fire' else (128, 128, 128)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated, name, (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        ok, jpeg = cv2.imencode('.jpg', annotated)
        if ok:
            m = CompressedImage()
            m.format = 'jpeg'
            m.data = jpeg.tobytes()
            self.frame_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.picam2.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
