import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import mediapipe as mp
import numpy as np

try:
    from tello_msgs.msg import FlightData
except ImportError:
    pass

class TelloGestureMachineNode(Node):
    def __init__(self):
        super().__init__('tello_gesture_machine_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        try:
            self.flight_data_sub = self.create_subscription(FlightData, '/flight_data', self.flight_data_cb, 10)
        except NameError:
            pass

        # ปรับ depth=1 เพื่อให้ภาพเรียลไทม์ที่สุด
        image_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_sub = self.create_subscription(Image, '/mono_py_driver/img_msg', self.image_cb, image_qos)

        # MediaPipe Hands Setup (จำกัด 1 มือ เพื่อไม่ให้สับสนเวลานับนิ้ว)
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.8, min_tracking_confidence=0.8)

        # ตัวแปรระบบ Drone Telemetry & Wind Stabilizer
        self.battery = 0
        self.roll = 0.0
        self.Kp_roll_twist = 0.06 

        self.get_logger().info("🛡️ ROS 2 Gesture Machine (Finger Counting + Wind Stabilizer) ACTIVE!")

    def flight_data_cb(self, msg):
        self.battery = msg.bat
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
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            results = self.hands.process(rgb_frame)
            
            hand_lr_speed = 0.0 # ความเร็วแกน Y (ซ้าย-ขวา)
            hand_fb_speed = 0.0 # ความเร็วแกน X (เดินหน้า-ถอยหลัง)
            fingers_up = 0
            current_gesture_text = "STABILIZING (HOVER)"

            if results.multi_hand_landmarks:
                hand_lms = results.multi_hand_landmarks[0]
                self.mp_draw.draw_landmarks(frame, hand_lms, self.mp_hands.HAND_CONNECTIONS)
                
                # 1. นับจำนวนนิ้วชี้ กลาง นาง ก้อย (เช็กว่าปลายนิ้วอยู่สูงกว่าข้อต่อ)
                tips = [8, 12, 16, 20]
                pips = [6, 10, 14, 18]
                
                for tip, pip in zip(tips, pips):
                    if hand_lms.landmark[tip].y < hand_lms.landmark[pip].y:
                        fingers_up += 1
                        
                # 2. ตรวจจับนิ้วโป้ง (Thumb) แบบใหม่ - วัดระยะห่างเทียบกับโคนนิ้วก้อย (จุดที่ 17)
                tip_x, tip_y = hand_lms.landmark[4].x, hand_lms.landmark[4].y       # ปลายนิ้วโป้ง
                joint_x, joint_y = hand_lms.landmark[3].x, hand_lms.landmark[3].y   # ข้อต่อนิ้วโป้ง
                pinky_x, pinky_y = hand_lms.landmark[17].x, hand_lms.landmark[17].y # โคนนิ้วก้อย

                # คำนวณระยะห่าง
                dist_tip_to_pinky = ((tip_x - pinky_x)**2 + (tip_y - pinky_y)**2)**0.5
                dist_joint_to_pinky = ((joint_x - pinky_x)**2 + (joint_y - pinky_y)**2)**0.5
                
                # ถ้าระยะจากปลายนิ้วโป้ง ไกลกว่า ระยะจากข้อต่อนิ้วโป้ง = แปลว่ากางนิ้วโป้งออก
                if dist_tip_to_pinky > dist_joint_to_pinky:
                    fingers_up += 1

                # 3. จัดการคำสั่งความเร็วตามจำนวนนิ้ว
                if fingers_up == 1:
                    hand_lr_speed = 0.5   # 1 นิ้ว = ซ้าย 
                    current_gesture_text = "1 FINGER : LEFT ⬅️"
                elif fingers_up == 2:
                    hand_lr_speed = -0.5  # 2 นิ้ว = ขวา 
                    current_gesture_text = "2 FINGERS: RIGHT ➡️"
                elif fingers_up == 3:
                    hand_fb_speed = 0.5   # 3 นิ้ว = เดินหน้า 
                    current_gesture_text = "3 FINGERS: FORWARD ⬆️"
                elif fingers_up == 4:
                    hand_fb_speed = -0.5  # 4 นิ้ว = ถอยหลัง 
                    current_gesture_text = "4 FINGERS: BACKWARD ⬇️"
                else:
                    # 0 หรือ 5 นิ้ว = จอดนิ่ง
                    current_gesture_text = f"{fingers_up} FINGERS: HOVER 🛑"

            # 4. ลอจิก Wind Stabilizer + ท่าทางมือ
            error_roll = 0 - self.roll
            stabilizer_speed_twist = error_roll * -self.Kp_roll_twist
            
            # รวมคำสั่งแกน Y (ซ้าย-ขวา) = ลม + มือ
            total_lr_speed = stabilizer_speed_twist + hand_lr_speed
            total_lr_speed = max(-1.00, min(1.00, total_lr_speed))

            # ส่งคำสั่งเข้าโดรนจริง
            cmd_msg = Twist()
            cmd_msg.linear.x = float(hand_fb_speed)
            cmd_msg.linear.y = float(total_lr_speed)
            self.cmd_vel_pub.publish(cmd_msg)

            # 5. HUD Display แบบยุทธวิธี
            cv2.rectangle(frame, (10, 10), (380, 145), (0, 0, 0), -1)
            
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, f"BATTERY : {self.battery}%", (20, 30), font, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, f"ROLL IMU: {self.roll:.1f} *", (20, 55), font, 0.5, (0, 255, 255), 2)
            cv2.putText(frame, f"CMD FB  : {hand_fb_speed:.2f} m/s", (20, 80), font, 0.5, (255, 100, 100), 2)
            cv2.putText(frame, f"TOTAL LR: {total_lr_speed:.2f} m/s", (20, 105), font, 0.5, (255, 165, 0), 2)
            
            # เปลี่ยนสีข้อความสถานะ: หากมีการสั่งงาน (1-4 นิ้ว) ให้เป็นสีเขียว
            text_color = (0, 255, 0) if (1 <= fingers_up <= 4) else (0, 255, 255)
            cv2.putText(frame, f"ACTION  : {current_gesture_text}", (20, 130), font, 0.5, text_color, 2)

            cv2.imshow("AI_TACTICAL_VISION", frame)
            cv2.setWindowProperty("AI_TACTICAL_VISION", cv2.WND_PROP_TOPMOST, 1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt

        except Exception as e:
            self.get_logger().error(f"Image Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TelloGestureMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist()) # เซฟตี้: หยุดโดรนทันทีเมื่อปิดโปรแกรม
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()