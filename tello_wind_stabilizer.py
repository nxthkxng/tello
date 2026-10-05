import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np

try:
    from tello_msgs.msg import FlightData
except ImportError:
    pass

class TelloWindStabilizer(Node):
    def __init__(self):
        super().__init__('tello_wind_stabilizer_node')

        # Publisher สำหรับส่งความเร็วแก้ลม (แกน Y: ซ้าย/ขวา)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber ดึงค่า Roll จากโดรน
        try:
            self.flight_data_sub = self.create_subscription(FlightData, '/flight_data', self.flight_data_cb, 10)
        except NameError:
            self.get_logger().warning("ไม่พบแพ็กเกจ tello_msgs")

        # Subscriber รับภาพเพื่อแสดงผล HUD เท่านั้น (ไม่สั่ง Takeoff)
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.image_sub = self.create_subscription(Image, '/mono_py_driver/img_msg', self.image_cb, image_qos)

        self.battery = 0
        self.roll = 0.0
        self.Kp_roll_twist = 0.05  # ค่าความไวในการสู้ลม
        self.latest_display_image = None

        self.get_logger().info("🛡️ ระบบ Wind-Stabilizer (สู้ลมอย่างเดียว) ACTIVE!")

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

            # ==========================================
            # ลอจิกสู้ลม (Wind Stabilizer Loop)
            # ==========================================
            error_roll = 0 - self.roll
            lr_speed_twist = error_roll * -self.Kp_roll_twist
            lr_speed_twist = max(-0.4, min(0.4, lr_speed_twist)) 

            # ส่งแรงสไลด์ตัวสู้ลมออกไปผ่าน Topic /cmd_vel
            cmd_msg = Twist()
            cmd_msg.linear.y = float(lr_speed_twist)
            self.cmd_vel_pub.publish(cmd_msg)

            # ==========================================
            # วาด Tactical HUD แสดงสถานะการสู้ลม
            # ==========================================
            cv2.rectangle(frame, (10, 10), (300, 100), (0, 0, 0), -1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            
            status_color = (0, 255, 0) if abs(self.roll) <= 3 else (0, 165, 255)
            
            cv2.putText(frame, f"BATTERY : {self.battery}%", (20, 30), font, font_scale, (0, 255, 0), 2)
            cv2.putText(frame, f"ROLL IMU: {self.roll:.1f} *", (20, 55), font, font_scale, status_color, 2)
            cv2.putText(frame, f"WIND CMD: {lr_speed_twist:.2f} m/s", (20, 80), font, font_scale, (255, 165, 0), 2)

            cv2.imshow("Tello Wind Stabilizer Monitor", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                raise KeyboardInterrupt

        except Exception as e:
            # แก้จาก sclf เป็น self เรียบร้อยแล้วครับ
            self.get_logger().error(f"Image Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TelloWindStabilizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("🛑 ปิดระบบ Wind Stabilizer เรียบร้อยครับ!")

if __name__ == '__main__':
    main()