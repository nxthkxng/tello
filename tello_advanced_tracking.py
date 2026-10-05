import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
import mediapipe as mp # 🛠️ นำเข้า MediaPipe

try:
    from tello_msgs.msg import FlightData
except ImportError:
    pass

class TelloAdvancedTrackerNode(Node):
    def __init__(self):
        super().__init__('tello_advanced_tracker_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        try:
            self.flight_data_sub = self.create_subscription(FlightData, '/flight_data', self.flight_data_cb, 10)
        except NameError:
            pass

        image_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_sub = self.create_subscription(Image, '/mono_py_driver/img_msg', self.image_cb, image_qos)

        # 🛠️ ตั้งค่า MediaPipe Face Detection
        self.mp_face_detection = mp.solutions.face_detection
        # model_selection=0 สำหรับกล้องระยะใกล้ (ไม่เกิน 2 เมตร), min_detection_confidence คือความมั่นใจขั้นต่ำ (0.7 = 70%)
        self.face_detection = self.mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.7)

        self.battery = 0
        self.height = 0
        self.roll = 0.0

        self.Kp_roll_twist = 0.04 
        
        self.target_locked = False

        self.get_logger().info("ROS 2 Advanced Tracker (MediaPipe Face Lock) ACTIVE!")

    def flight_data_cb(self, msg):
        self.battery = msg.bat
        self.height = msg.h
        self.roll = msg.roll

    def image_cb(self, msg):
        try:
            raw_data = bytes(msg.data)
            img_1d = np.frombuffer(raw_data, dtype=np.uint8).copy()
            
            if 'mono' in msg.encoding or '8UC1' in msg.encoding:
                frame = img_1d.reshape((msg.height, msg.width))
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                frame = img_1d.reshape((msg.height, msg.width, 3))
                if 'rgb' in msg.encoding:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            frame = cv2.resize(frame, (640, 480))
            
            # 🛠️ แปลงสีเพื่อส่งให้ MediaPipe ประมวลผล
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.face_detection.process(rgb_frame)

            # 1. ลอจิกสู้ลม (รักษาสมดุลซ้าย-ขวาตลอดเวลา)
            error_roll = 0 - self.roll
            lr_speed_twist = max(-0.45, min(0.45, error_roll * -self.Kp_roll_twist))

            yaw_speed_twist = 0.0 
            ud_speed_twist = 0.0  
            fb_speed_twist = 0.0  
            
            status_text = ""
            status_color = (255, 255, 255)

            # วาดกรอบเป้าหมายตรงกลาง (Deadband Zone)
            cv2.rectangle(frame, (280, 200), (360, 280), (255, 255, 0), 1)
            cv2.circle(frame, (320, 240), 2, (0, 0, 255), -1)

            # 2. ตรวจสอบว่า MediaPipe เจอใบหน้าหรือไม่
            if results.detections:
                # 🟢 กรณีที่ 1: เจอหน้าเป้าหมายแล้ว (ล็อกเป้า)
                self.target_locked = True
                status_text = "TARGET LOCKED" # ลบ Emoji ออกเพื่อแก้ปัญหา ???? บนหน้าจอ
                status_color = (0, 255, 0)
                
                # ดึงข้อมูลใบหน้าแรกที่เจอมาใช้งาน
                detection = results.detections[0]
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, _ = frame.shape
                
                # แปลงพิกัดแบบ Relative เป็นค่า Pixel จริงบนหน้าจอ
                x = int(bboxC.xmin * iw)
                y = int(bboxC.ymin * ih)
                w = int(bboxC.width * iw)
                h = int(bboxC.height * ih)
                
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cx, cy = x + w // 2, y + h // 2
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                
                # ลอจิกหันหน้าและขึ้นลงตาม
                if cx < 280 or cx > 360:
                    yaw_speed_twist = max(-0.5, min(0.5, (cx - 320) * -0.002))
                if cy < 200 or cy > 280:
                    ud_speed_twist = max(-0.4, min(0.4, (240 - cy) * 0.003))

                # ลอจิกรักษาระยะห่าง 1.5 เมตร
                area = w * h
                target_area = 18000 
                
                if area < target_area - 3000:
                    fb_speed_twist = 0.15 # ถอยมาไกลไป บินเข้าหา
                elif area > target_area + 3000:
                    fb_speed_twist = -0.15 # เข้ามาใกล้ไป ถอยหนี

                cv2.putText(frame, f"AREA: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            else:
                # 🟡 กรณีที่ 2: ยังไม่เจอหน้าเป้าหมาย
                if not self.target_locked:
                    if self.height < 200: 
                        ud_speed_twist = 0.02 
                        status_text = "SCANNING (ASCENDING)" 
                        status_color = (0, 255, 255) 
                    else:
                        ud_speed_twist = 0.0
                        status_text = "SCANNING (MAX HEIGHT)" 
                        status_color = (0, 165, 255) 
                else:
                    # 🔴 2.2 เคยล็อกเป้าได้แล้ว แต่เป้าหมายหลุดกล้อง
                    status_text = "TARGET LOST (HOVER)" 
                    status_color = (0, 0, 255) 

            # ส่งคำสั่งไปที่โดรน
            cmd_msg = Twist()
            cmd_msg.linear.x = float(fb_speed_twist)   
            cmd_msg.linear.y = float(lr_speed_twist)   
            cmd_msg.linear.z = float(ud_speed_twist)   
            cmd_msg.angular.z = float(yaw_speed_twist) 
            self.cmd_vel_pub.publish(cmd_msg)

            # HUD Dashboard
            cv2.rectangle(frame, (10, 10), (320, 150), (0, 0, 0), -1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, f"BATTERY : {self.battery}%", (20, 30), font, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"HEIGHT  : {self.height} cm", (20, 55), font, 0.5, (255, 255, 255), 2)
            
            color = (0, 255, 0) if (yaw_speed_twist != 0 or ud_speed_twist != 0 or fb_speed_twist != 0) else (150, 150, 150)
            cv2.putText(frame, f"YAW CMD : {yaw_speed_twist:.2f}", (20, 90), font, 0.5, color, 2)
            cv2.putText(frame, f"UP/DN CMD: {ud_speed_twist:.2f}", (20, 115), font, 0.5, color, 2)
            
            cv2.putText(frame, status_text, (20, 140), font, 0.55, status_color, 2)

            cv2.imshow("Tello Tactical Tracking Hub", frame)
            cv2.setWindowProperty("Tello Tactical Tracking Hub", cv2.WND_PROP_TOPMOST, 1)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt

        except Exception as e:
            self.get_logger().error(f"Image Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TelloAdvancedTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist()) 
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()