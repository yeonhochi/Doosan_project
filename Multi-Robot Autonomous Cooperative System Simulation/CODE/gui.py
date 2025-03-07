import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import tkinter as tk
from threading import Thread, Timer
from PIL import Image as PILImage, ImageTk
import numpy as np
import pygame
import time

class ImageListener(Node):
    def __init__(self, root):
        super().__init__('image_listener')
        self.subscription1 = self.create_subscription(
            CompressedImage, '/tb3_1/camera/image_compressed', self.image_callback1, 10)
        self.subscription2 = self.create_subscription(
            CompressedImage, '/tb3_2/camera/image_compressed', self.image_callback2, 10)
        self.cell_subscription = self.create_subscription(
            String, '/start_detect', self.cell_callback, 10)
        
        self.bridge = CvBridge()
        self.emergency_publisher = self.create_publisher(String, '/emergency', 10)
        self.found_publisher = self.create_publisher(String, '/found', 10)
        
        self.model = YOLO('yolov8n.pt')
        
        self.root = root
        self.root.title("실시간 감시 System")
        
        self.frame = tk.Frame(self.root)
        self.frame.pack()

        # 상태 라벨
        self.mode_label = tk.Label(self.frame, text="순찰 모드", font=("Arial", 16), fg="black")
        self.mode_label.grid(row=0, column=0, columnspan=2, sticky="nsew")

        self.status_label1 = tk.Label(self.frame, text="상태: 순찰 중", font=("Arial", 14), fg="black")
        self.status_label1.grid(row=1, column=0, sticky="nsew")

        self.status_label2 = tk.Label(self.frame, text="상태: 순찰 중", font=("Arial", 14), fg="black")
        self.status_label2.grid(row=1, column=1, sticky="nsew")

        self.emergency_button = tk.Button(self.frame, text="출동", font=("Arial", 14), command=self.send_emergency)
        self.emergency_button.grid(row=2, column=0, columnspan=2, pady=10)

        self.canvas1 = tk.Canvas(self.frame, width=640, height=480, highlightthickness=0)
        self.canvas1.grid(row=3, column=0, sticky="nsew")

        self.canvas2 = tk.Canvas(self.frame, width=640, height=480, highlightthickness=0)
        self.canvas2.grid(row=3, column=1, sticky="nsew")

        self.frame.rowconfigure(3, weight=1)
        self.frame.columnconfigure(0, weight=1)
        self.frame.columnconfigure(1, weight=1)

        self.in_tracking_mode = False
        self.cells = set()
        self.cell_timers = {}
        self.reached_waypoints = {13: False, 14: False}
        
        self.log_label = tk.Label(self.frame, text="", font=("Arial", 20), fg="blue")
        self.log_label.grid(row=4, column=0, columnspan=2)
        self.found_published = False
        
        pygame.mixer.init()
    
    
    def play_sound(self, sound_file):
        pygame.mixer.music.load(sound_file)
        pygame.mixer.music.play()
        time.sleep(10)
        pygame.mixer.music.stop()
    
    
    def send_emergency(self):
        msg = String()
        msg.data = "출동"
        self.emergency_publisher.publish(msg)
        self.get_logger().info("로봇 출동!")

        self.in_tracking_mode = True
        self.reached_waypoints = {13: False, 14: False}

        bg_color = "#ffcccc"
        self.root.config(bg=bg_color)
        self.frame.config(bg=bg_color)

        self.mode_label.config(text="추적 모드", fg="red", bg=bg_color)
        self.status_label1.config(text="상태: 추적 중", fg="red", bg=bg_color)
        self.status_label2.config(text="상태: 추적 중", fg="red", bg=bg_color)
        self.emergency_button.config(bg=bg_color)
        self.root.after(0, lambda: self.emergency_button.grid_forget())

        self.root.update_idletasks()

        Thread(target=self.play_sound, args=("emergency.mp3",)).start()


    def cell_callback(self, msg):
        try:
            cell_num = int(msg.data)

            if cell_num not in self.cells:
                self.cells.add(cell_num)

                if cell_num in self.cell_timers:
                    self.cell_timers[cell_num].cancel()
                    del self.cell_timers[cell_num]

                self.cell_timers[cell_num] = Timer(2.0, self.remove_cell, [cell_num])
                self.cell_timers[cell_num].start()

                # 13번, 14번 도착 체크
                if cell_num in self.reached_waypoints:
                    self.reached_waypoints[cell_num] = True
                    self.get_logger().info(f"로봇이 {cell_num}번 위치에 도착!")
        except ValueError:
            self.get_logger().error(f"Invalid cell number received: {msg.data}")


    def remove_cell(self, cell_num):
        """ 셀을 제거할 때 안정적으로 삭제 """
        if cell_num in self.cells:
            self.cells.discard(cell_num)
        if cell_num in self.cell_timers:
            self.cell_timers[cell_num].cancel()
            del self.cell_timers[cell_num]


    def update_log_label(self, message):
        """ UI 로그 라벨을 업데이트하는 함수 """
        self.log_label.config(text=message)
        self.root.update_idletasks()


    def process_images(self, msg1, msg2):
        try:
            cv_image1 = self.bridge.compressed_imgmsg_to_cv2(msg1, 'bgr8')
            cv_image2 = self.bridge.compressed_imgmsg_to_cv2(msg2, 'bgr8')

            results = self.model([cv_image1, cv_image2], classes=[0])

            person_detected_robot1 = self.process_result(cv_image1, results[0], self.status_label1, self.canvas1, 1, 6)
            person_detected_robot2 = self.process_result(cv_image2, results[1], self.status_label2, self.canvas2, 7, 12)

            if self.in_tracking_mode and all(self.reached_waypoints.values()) and not self.found_published:
                detected_robots = None
                if person_detected_robot1:
                    detected_robots = '1'
                if person_detected_robot2:
                    detected_robots = '2'

                if detected_robots:
                    found_msg = String()
                    found_msg.data = detected_robots
                    self.found_publisher.publish(found_msg)
                    self.get_logger().info(f"탈옥수 발견! 보고 로봇: {found_msg.data}번 로봇")

                    # UI 로그 업데이트
                    self.root.after(0, lambda: self.log_label.config(text=f"탈옥수 발견! 보고 로봇: {found_msg.data}번 로봇"))
                    
                    Thread(target=self.play_sound, args=("found.mp3",)).start()

                    # 중복 전송 방지
                    self.found_published = True

        except Exception as e:
            self.get_logger().error(f"Failed to process images: {e}")

    
    def process_result(self, cv_image, result, status_label, canvas, min_cell, max_cell):
        person_detected = any(result.boxes)
        
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(cv_image, "Detected", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            person_detected = True
        
        detected_cells = [cell for cell in self.cells if min_cell <= cell <= max_cell]
        if detected_cells:
            status = f"{', '.join(map(str, detected_cells))}번 방 이상 무" if person_detected else f"{', '.join(map(str, detected_cells))}번 방 탈옥수 발생"
            color = "green" if person_detected else "red"
        else:
            status = "순찰 중"
            color = "black"
        
        self.root.after(0, lambda: status_label.config(text=f"상태: {status}", fg=color))
        
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img = PILImage.fromarray(cv_image)
        img = ImageTk.PhotoImage(image=img)
        
        self.root.after(0, lambda: self.update_canvas(canvas, img))
        return person_detected
    
    def image_callback1(self, msg):
        self.latest_msg1 = msg
        if hasattr(self, 'latest_msg2'):
            self.process_images(self.latest_msg1, self.latest_msg2)
    
    def image_callback2(self, msg):
        self.latest_msg2 = msg
        if hasattr(self, 'latest_msg1'):
            self.process_images(self.latest_msg1, self.latest_msg2)
    
    def update_canvas(self, canvas, img):
        canvas.create_image(0, 0, anchor=tk.NW, image=img)
        canvas.image = img


def main(args=None):
    rclpy.init(args=args)
    
    root = tk.Tk()
    node = ImageListener(root)
    
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    
    root.mainloop()
    
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
