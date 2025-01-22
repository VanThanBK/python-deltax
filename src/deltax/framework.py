from PySide6.QtCore import QObject, Signal, Slot, QTimer
from .robot import Robot
from .encoder import Encoder
from .camera import Camera
from .conveyor import Conveyor
import cv2
import numpy as np
import threading
import time

class DeltaXFramework(QObject):
    # Signals
    robot_position_changed = Signal(list)  # Emit khi vị trí robot thay đổi
    camera_frame_ready = Signal(np.ndarray)  # Emit khi có frame mới từ camera
    object_detected = Signal(str, tuple)  # Emit khi phát hiện vật (class_name, position)
    encoder_value_changed = Signal(float)  # Emit khi giá trị encoder thay đổi
    
    def __init__(self):
        super().__init__()
        self.robot = None
        self.encoder = None
        self.camera = None
        self.conveyor = None
        self.is_running = False
        
        # Timer để cập nhật vị trí robot
        self._position_timer = QTimer()
        self._position_timer.timeout.connect(self._update_robot_position)
        
    def connect_robot(self, port=None, model=Robot.X1):
        """Kết nối với robot"""
        if port is None:
            # Tự động tìm robot
            ports = Robot.scan_ports()
            if not ports:
                raise Exception("No DeltaX robot found!")
            port = ports[0]
            
        self.robot = Robot(port=port, model=model)
        if not self.robot.connect():
            raise Exception(f"Failed to connect to robot on port {port}")
            
        # Bắt đầu theo dõi vị trí
        self._position_timer.start(100)  # Cập nhật mỗi 100ms
        return self
        
    def start_camera(self, camera_id=0):
        """Khởi động camera"""
        self.camera = Camera(camera_id)
        self.camera.start()
        
        # Kết nối signal
        self.camera.frame_ready.connect(
            lambda frame: self.camera_frame_ready.emit(frame)
        )
        return self
        
    def start_encoder(self, port=None):
        """Khởi động encoder"""
        if port is None:
            # Tự động tìm encoder
            ports = Encoder.scan_ports()
            if not ports:
                raise Exception("No encoder found!")
            port = ports[0]
            
        self.encoder = Encoder(port=port)
        if not self.encoder.connect():
            raise Exception(f"Failed to connect encoder on port {port}")
            
        # Kết nối signal
        self.encoder.position_changed.connect(
            lambda pos: self.encoder_value_changed.emit(pos)
        )
        return self
        
    def start(self):
        """Bắt đầu hoạt động"""
        self.is_running = True
        return self
        
    def stop(self):
        """Dừng hoạt động"""
        self.is_running = False
        if self.camera:
            self.camera.stop()
        if self.robot:
            self.robot.disconnect()
        if self.encoder:
            self.encoder.disconnect()
            
    @Slot()
    def _update_robot_position(self):
        """Cập nhật vị trí robot"""
        if self.robot and self.robot.isResponded():
            self.robot.syncPosition(is_wait=False)
            pos = self.robot.position()
            self.robot_position_changed.emit(pos)
            
    def move_to_object(self, object_position, z_height=-300):
        """Di chuyển robot đến vị trí vật thể"""
        if not self.robot:
            return
            
        x, y = object_position
        self.robot.move(x=x, y=y, z=z_height)
        return self
        
    def pick_and_place(self, pick_pos, place_pos, z_height=-300, z_up=-200):
        """Thực hiện chu trình gắp và đặt"""
        if not self.robot:
            return
            
        # Kiểm tra tính năng end effector
        if not self.robot.check_feature('end_effector'):
            print("Warning: End effector not supported on this robot model")
            return self
            
        # Di chuyển đến vị trí gắp
        self.robot.move(x=pick_pos[0], y=pick_pos[1], z=z_up)
        self.robot.move(z=z_height)
        
        # Bật bơm hút
        self.robot.controlEndEffector(value=Robot.ON)
        
        # Nâng lên
        self.robot.move(z=z_up)
        
        # Di chuyển đến vị trí đặt
        self.robot.move(x=place_pos[0], y=place_pos[1])
        self.robot.move(z=z_height)
        
        # Tắt bơm hút
        self.robot.controlEndEffector(value=Robot.OFF)
        
        # Nâng lên
        self.robot.move(z=z_up)
        return self
        
    def get_encoder_position(self):
        """Đọc vị trí encoder"""
        if self.encoder:
            return self.encoder.read_position()
        return 0.0 