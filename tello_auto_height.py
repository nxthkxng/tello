import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
import subprocess
import os

try:
    from tello_msgs.msg import FlightData
    from tello_msgs.srv import TelloAction
except ImportError:
    FlightData = None
    TelloAction = None

class TelloMainGUI(Node):
    def __init__(self):
        super().__init__('tello_main_gui_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        if TelloAction is not None:
            self.action_client = self.create_client(TelloAction, '/tello_action')
            while not self.action_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('⏳ กำลังรอเชื่อมต่อ Service /tello_action...')
        else:
            self.action_client = None
            self.get_logger().error('❌ ไม่พบ tello_msgs/srv/TelloAction!')

        self.flight_data_sub = None
        self.battery = 0
        self.temp = 0.0
        self.height = 0
        self.speed = 0.5
        self.roll = 0.0
        self.pitch = 0.0
        self.standby_anti_drift = False
        self.takeoff_timer = None
        
        # --- ตัวแปรสำหรับระบบควบคุมความสูง (Auto Height) ---
        self.target_height = None
        self.Kp_height = 0.015
        self.Kp_roll_twist = 0.02
        self.Kp_pitch_twist = 0.02
        # --------------------------------------------------

        self.anti_drift_loop_timer = self.create_timer(0.05, self.anti_drift_process)

        self.init_subscriptions()

    def init_subscriptions(self):
        try:
            if self.flight_data_sub is not None:
                self.destroy_subscription(self.flight_data_sub)
        except Exception:
            pass

        if FlightData is not None:
            try:
                self.flight_data_sub = self.create_subscription(FlightData, '/flight_data', self.flight_data_callback, 10)
            except Exception as e:
                pass

    def flight_data_callback(self, msg):
        try:
            self.battery = getattr(msg, 'bat', 0)
            templ = getattr(msg, 'templ', 0.0)
            temph = getattr(msg, 'temph', 0.0)
            self.temp = (templ + temph) / 2.0
            self.height = getattr(msg, 'h', 0)
            self.roll = getattr(msg, 'roll', 0.0)
            self.pitch = getattr(msg, 'pitch', 0.0)
        except Exception:
            pass

    def send_action(self, cmd_str):
        if self.action_client and self.action_client.service_is_ready():
            req = TelloAction.Request()
            req.cmd = cmd_str
            self.action_client.call_async(req)
            self.get_logger().info(f"📤 ส่งคำสั่ง Service [{cmd_str}] สำเร็จ")
        else:
            self.get_logger().error(f"❌ Service /tello_action ยังไม่พร้อมใช้งาน!")

    def send_takeoff(self, target_h):
        self.target_height = target_h
        self.send_action('takeoff')
        self.standby_anti_drift = False
        if self.takeoff_timer is not None:
            self.takeoff_timer.cancel()
        # หน่วงเวลา 4 วินาทีให้โดรนบินพ้นพื้น แล้วเปิดระบบต้านลม + ล็อกความสูงตามที่กรอกไว้
        self.takeoff_timer = self.create_timer(4.0, self.enable_anti_drift)
        self.get_logger().info(f"🛫 Takeoff! จะเริ่มรักษาระดับความสูงที่ {target_h} cm และต้านลมอัตโนมัติใน 4 วินาที...")

    def send_land(self):
        self.standby_anti_drift = False
        self.target_height = None
        self.send_action('land')
        self.create_timer(0.5, lambda: self.send_action('land'))

    def enable_anti_drift(self):
        self.standby_anti_drift = True
        self.get_logger().info(f"🛡 ระบบกันลมและรักษาระดับความสูง (เป้าหมาย {self.target_height} cm) เริ่มทำงาน!")
        if self.takeoff_timer is not None:
            self.takeoff_timer.cancel()
            self.takeoff_timer = None

    def anti_drift_process(self):
        if self.standby_anti_drift:
            twist = Twist()
            # 1. ระบบกันลม
            twist.linear.y = float(max(-0.4, min(0.4, self.roll * self.Kp_roll_twist)))
            twist.linear.x = float(max(-0.4, min(0.4, self.pitch * self.Kp_pitch_twist)))
            
            # 2. ระบบรักษาระดับความสูงอัตโนมัติ (Auto Height P-Controller)
            if self.target_height is not None:
                error_h = self.target_height - self.height
                if abs(error_h) > 5: # Deadband 5 cm ป้องกันการกระตุก
                    speed_z = error_h * self.Kp_height
                    twist.linear.z = float(max(-0.5, min(0.5, speed_z)))
                    
            self.cmd_vel_pub.publish(twist)

    def stop_drone(self):
        self.standby_anti_drift = False
        self.target_height = None
        self.cmd_vel_pub.publish(Twist())

    def send_cmd(self, x, y, z, yaw):
        self.standby_anti_drift = False
        self.target_height = None
        msg = Twist()
        msg.linear.x = float(x) * self.speed
        msg.linear.y = float(y) * self.speed
        msg.linear.z = float(z) * self.speed
        msg.angular.z = float(yaw) * self.speed
        self.cmd_vel_pub.publish(msg)

    def safe_emergency_land(self):
        self.get_logger().warn("🚨 ยกเลิกภารกิจ: เบรกฉุกเฉินและค่อยๆ ร่อนลงจอดอย่างปลอดภัย! 🚨")
        self.standby_anti_drift = False
        self.target_height = None
        self.cmd_vel_pub.publish(Twist())
        self.send_land()


def main():
    rclpy.init()
    node = TelloMainGUI()

    root = tk.Tk()
    root.title("Tello Tactical Control Hub")
    root.geometry("480x880")
    root.configure(bg="#1e272e")

    tk.Label(root, text="ROS 2 Tello Control Hub", font=("Helvetica", 16, "bold"), fg="#00d8d6", bg="#1e272e").pack(pady=15)

    status_frame = tk.Frame(root, bg="#1e272e")
    status_frame.pack(pady=5)

    lbl_bat = tk.Label(status_frame, text="🔋 BAT: 0%", font=("Helvetica", 11, "bold"), fg="#0be881", bg="#1e272e")
    lbl_bat.grid(row=0, column=0, padx=10)
    lbl_temp = tk.Label(status_frame, text="🌡️ TMP: 0°C", font=("Helvetica", 11, "bold"), fg="#ffdd59", bg="#1e272e")
    lbl_temp.grid(row=0, column=1, padx=10)
    lbl_height = tk.Label(status_frame, text="📏 HGT: 0 cm", font=("Helvetica", 11, "bold"), fg="#4bcffa", bg="#1e272e")
    lbl_height.grid(row=0, column=2, padx=10)

    lbl_anti_drift = tk.Label(status_frame, text="🛡️ ANTI-DRIFT: OFF", font=("Helvetica", 10, "bold"), fg="#808e9b", bg="#1e272e")
    lbl_anti_drift.grid(row=1, column=0, columnspan=3, pady=8)

    def update_gui_loop():
        try:
            rclpy.spin_once(node, timeout_sec=0.01)
        except Exception:
            pass

        lbl_bat.config(text=f"🔋 BAT: {node.battery}%")
        lbl_temp.config(text=f"🌡️ TMP: {node.temp:.1f}°C")
        lbl_height.config(text=f"📏 HGT: {node.height} cm")

        if node.standby_anti_drift:
            if node.target_height is not None:
                lbl_anti_drift.config(text=f"🛡️ สู้ลม & ล็อกความสูง (🎯 {int(node.target_height)} cm)", fg="#0be881")
            else:
                lbl_anti_drift.config(text="🛡️ ANTI-DRIFT: ACTIVE (HOLDING POSITION)", fg="#0be881")
        else:
            lbl_anti_drift.config(text="🛡️ ANTI-DRIFT: OFF / WAITING", fg="#808e9b")

        root.after(50, update_gui_loop)

    update_gui_loop()

    # =======================================================
    # ส่วนเพิ่ม: ช่องกรอกความสูงเป้าหมายก่อนบิน
    # =======================================================
    height_setup_frame = tk.Frame(root, bg="#1e272e")
    height_setup_frame.pack(pady=5)
    
    tk.Label(height_setup_frame, text="🎯 ตั้งความสูงเป้าหมายก่อนบิน (cm):", font=("Arial", 11, "bold"), fg="#fbc531", bg="#1e272e").grid(row=0, column=0, padx=5)
    
    entry_target_h = tk.Entry(height_setup_frame, font=("Arial", 11, "bold"), width=8, justify="center")
    entry_target_h.insert(0, "100") # ค่าเริ่มต้น 100 ซม.
    entry_target_h.grid(row=0, column=1, padx=5)
    
    def on_takeoff_click():
        try:
            target_h = float(entry_target_h.get())
        except ValueError:
            target_h = 100.0 # ค่าสำรองถ้ากรอกผิดพลาด
        node.send_takeoff(target_h)
    # =======================================================

    main_ctrl_frame = tk.Frame(root, bg="#1e272e")
    main_ctrl_frame.pack(pady=10)

    # ปุ่ม Takeoff เรียกใช้งานฟังก์ชัน on_takeoff_click ที่อ่านค่าจากช่องกรอก
    tk.Button(main_ctrl_frame, text=" บินขึ้น (Takeoff)", font=("Arial", 11, "bold"), bg="#05c46b", fg="white", width=14, command=on_takeoff_click).grid(row=0, column=0, padx=5, pady=3)
    tk.Button(main_ctrl_frame, text=" ลงจอด (Land)", font=("Arial", 11, "bold"), bg="#ff3f34", fg="white", width=14, command=node.send_land).grid(row=0, column=1, padx=5, pady=3)
    tk.Button(main_ctrl_frame, text="🛑 หยุดลอยตัว (Hover)", font=("Arial", 11, "bold"), bg="#ffa801", fg="white", width=30, command=node.stop_drone).grid(row=1, column=0, columnspan=2, pady=5)

    dir_frame = tk.Frame(root, bg="#1e272e")
    dir_frame.pack(pady=10)
    btn_style = {"font": ("Arial", 11, "bold"), "bg": "#3c40c6", "fg": "white", "width": 10, "height": 1}
    
    tk.Button(dir_frame, text="⬆️ หน้า", command=lambda: node.send_cmd(3, 0, 0, 0), **btn_style).grid(row=0, column=1, pady=3)
    tk.Button(dir_frame, text="⬅️ ซ้าย", command=lambda: node.send_cmd(0, 3, 0, 0), **btn_style).grid(row=1, column=0, padx=5)
    tk.Button(dir_frame, text="⬇️ หลัง", command=lambda: node.send_cmd(-3, 0, 0, 0), **btn_style).grid(row=1, column=1, pady=3)
    tk.Button(dir_frame, text="➡️ ขวา", command=lambda: node.send_cmd(0, -3, 0, 0), **btn_style).grid(row=1, column=2, padx=5)

    mode_frame = tk.LabelFrame(root, text="🧠 ระบบผู้ช่วยการบิน (AI & Sensors)", font=("Arial", 11, "bold"), bg="#1e272e", fg="#00d8d6", padx=10, pady=5)
    mode_frame.pack(pady=10, fill=tk.X, padx=20)

    btn_mode_style = {"font": ("Arial", 10, "bold"), "bg": "#575fcf", "fg": "white", "width": 38, "pady": 5}
    active_mode = None

    def toggle_ai_mode(mode_id, script_name):
        nonlocal active_mode
        if active_mode == mode_id:
            node.get_logger().info(f"🛑 กำลังปิด AI Module: {script_name}")
            os.system(f"pkill -f {script_name}")
            active_mode = None
            node.standby_anti_drift = True
            btn_wind.config(state=tk.NORMAL, text="🌪️ เปิดโหมด: สู้ลมอัตโนมัติ", bg="#575fcf")
            btn_gest.config(state=tk.NORMAL, text="🤖 เปิดโหมด: คุมด้วยมือ (Gesture)", bg="#575fcf")
            btn_track.config(state=tk.NORMAL, text="🎯 เปิดโหมด: ล็อกเป้าหมาย (Tracking)", bg="#575fcf")
        else:
            active_mode = mode_id
            node.standby_anti_drift = False
            node.target_height = None
            node.get_logger().info(f"🚀 เปิด AI Module: {script_name}")
            full_command = f"gnome-terminal -- bash -c 'source ~/tello_ws/install/setup.bash && python3 {script_name}; exec bash'"
            subprocess.Popen(full_command, shell=True)
            btn_wind.config(state=tk.DISABLED, bg="#2f3640")
            btn_gest.config(state=tk.DISABLED, bg="#2f3640")
            btn_track.config(state=tk.DISABLED, bg="#2f3640")
            if mode_id == "wind":
                btn_wind.config(state=tk.NORMAL, text="🔴 ปิดโหมด: สู้ลมอัตโนมัติ (OFF)", bg="#ff4757")
            elif mode_id == "gest":
                btn_gest.config(state=tk.NORMAL, text="🔴 ปิดโหมด: คุมด้วยมือ (OFF)", bg="#ff4757")
            elif mode_id == "track":
                btn_track.config(state=tk.NORMAL, text="🔴 ปิดโหมด: ล็อกเป้าหมาย (OFF)", bg="#ff4757")

    btn_wind = tk.Button(mode_frame, text=" เปิดโหมด: สู้ลมอัตโนมัติ", **btn_mode_style)
    btn_gest = tk.Button(mode_frame, text=" เปิดโหมด: คุมด้วยมือ (Gesture)", **btn_mode_style)
    btn_track = tk.Button(mode_frame, text=" เปิดโหมด: ล็อกเป้าหมาย (Tracking)", **btn_mode_style)

    btn_wind.config(command=lambda: toggle_ai_mode("wind", "tello_wind_stabilizer.py"))
    btn_gest.config(command=lambda: toggle_ai_mode("gest", "tello_Machine.py"))
    btn_track.config(command=lambda: toggle_ai_mode("track", "tello_advanced_tracking.py"))

    btn_wind.pack(pady=3)
    btn_gest.pack(pady=3)
    btn_track.pack(pady=3)

    def trigger_emergency():
        node.safe_emergency_land()
        nonlocal active_mode
        if active_mode:
            if active_mode == "wind": target_script = "tello_wind_stabilizer.py"
            elif active_mode == "gest": target_script = "tello_Machine.py"
            elif active_mode == "track": target_script = "tello_advanced_tracking.py"
            os.system(f"pkill -f {target_script}")
            active_mode = None
            btn_wind.config(state=tk.NORMAL, text=" เปิดโหมด: สู้ลมอัตโนมัติ", bg="#575fcf")
            btn_gest.config(state=tk.NORMAL, text=" เปิดโหมด: คุมด้วยมือ (Gesture)", bg="#575fcf")
            btn_track.config(state=tk.NORMAL, text=" เปิดโหมด: ล็อกเป้าหมาย (Tracking)", bg="#575fcf")

    tk.Button(root, text="🚨 ฉุกเฉิน: เบรกและค่อยๆ ร่อนลง (SAFE LAND) 🚨",
              font=("Arial", 11, "bold"), bg="#ff1e56", fg="white",
              activebackground="#c0392b", activeforeground="white",
              height=2, command=trigger_emergency).pack(fill=tk.X, padx=20, pady=10)

    def on_closing():
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == '__main__':
    main()