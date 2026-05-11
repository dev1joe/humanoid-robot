import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from ultralytics import YOLO
import cv2

class BallDetectorNode(Node):
    def __init__(self):
        super().__init__('ball_detector_node')

        self.publisher_ = self.create_publisher(Point, '/ball_position', 10)
        self.dir_publisher = self.create_publisher(String, '/ball_direction', 10)

        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLOv8 model loaded')

        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.timer = self.create_timer(0.1, self.detect_ball)

        self.img_w = 640
        self.img_h = 480

        self.center_threshold = 0.15
        self.kick_threshold   = 250

    def get_direction(self, norm_x, norm_y, bbox_w):
        """بيرجع الاتجاه المطلوب بناءً على مكان الكورة"""
        if bbox_w > self.kick_threshold:
            return "KICK"
        elif norm_x < -self.center_threshold:
            return "TURN LEFT"
        elif norm_x > self.center_threshold:
            return "TURN RIGHT"
        elif norm_y < -self.center_threshold:
            return "LOOK UP"
        else:
            return "MOVE FORWARD"

    def draw_hud(self, frame, cx, cy, norm_x, norm_y, bbox_w, direction, x1, y1, x2, y2):
        """بيرسم كل الـ info على الـ frame"""

        color_map = {
            "KICK":         (0, 0, 255),
            "TURN LEFT":    (255, 100, 0),
            "TURN RIGHT":   (0, 100, 255),
            "MOVE FORWARD": (0, 255, 0),
            "LOOK UP":      (255, 255, 0),
        }
        color = color_map.get(direction, (255, 255, 255))

        # Bounding box + center dot
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        cv2.circle(frame, (int(cx), int(cy)), 6, (0, 0, 255), -1)

        img_cx = self.img_w // 2
        img_cy = self.img_h // 2
        cv2.line(frame, (img_cx, img_cy), (int(cx), int(cy)), (200, 200, 200), 1)
        cv2.circle(frame, (img_cx, img_cy), 4, (200, 200, 200), -1)

        cv2.putText(frame, f'Ball X: {norm_x:+.2f}  Y: {norm_y:+.2f}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        cv2.putText(frame, f'Pixel: ({int(cx)}, {int(cy)})',
                    (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        cv2.putText(frame, f'BBox W: {bbox_w:.0f}px',
                    (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        # ---- Direction banner ----
        cv2.rectangle(frame, (0, self.img_h - 45), (self.img_w, self.img_h), (30, 30, 30), -1)
        cv2.putText(frame, f'>> {direction} <<',
                    (self.img_w // 2 - 90, self.img_h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, color, 2)

        # ---- Mini compass/arrow ----
        arrow_ox, arrow_oy = self.img_w - 60, 60
        arrow_dx = int(norm_x * 40)
        arrow_dy = int(norm_y * 40)
        cv2.circle(frame, (arrow_ox, arrow_oy), 35, (50, 50, 50), -1)
        cv2.circle(frame, (arrow_ox, arrow_oy), 35, (150, 150, 150), 1)
        cv2.arrowedLine(frame,
                        (arrow_ox, arrow_oy),
                        (arrow_ox + arrow_dx, arrow_oy + arrow_dy),
                        color, 2, tipLength=0.4)

    def detect_ball(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame')
            return

        results = self.model(frame, verbose=False)
        ball_found = False

        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                label = self.model.names[cls]

                if label == 'sports ball':
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    bbox_w = x2 - x1

                    norm_x = (cx - self.img_w / 2) / (self.img_w / 2)
                    norm_y = (cy - self.img_h / 2) / (self.img_h / 2)

                    direction = self.get_direction(norm_x, norm_y, bbox_w)

                    # Publish position
                    msg = Point()
                    msg.x = norm_x
                    msg.y = norm_y
                    msg.z = float(bbox_w)
                    self.publisher_.publish(msg)

                    # Publish direction
                    dir_msg = String()
                    dir_msg.data = direction
                    self.dir_publisher.publish(dir_msg)

                    self.get_logger().info(f'[{direction}] norm=({norm_x:+.2f}, {norm_y:+.2f}) bbox_w={bbox_w:.0f}')

                    self.draw_hud(frame, cx, cy, norm_x, norm_y, bbox_w, direction, x1, y1, x2, y2)
                    ball_found = True
                    break

        if not ball_found:
            cv2.putText(frame, 'NO BALL DETECTED',
                        (self.img_w // 2 - 120, self.img_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            dir_msg = String()
            dir_msg.data = "SEARCH"
            self.dir_publisher.publish(dir_msg)

        cv2.imshow('Ball Detection', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()