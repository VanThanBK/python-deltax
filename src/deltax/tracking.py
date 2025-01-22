from PySide6.QtCore import QObject, Signal, QTimer
from dataclasses import dataclass
import numpy as np
import cv2

@dataclass
class TrackedObject:
    """Class chứa thông tin về đối tượng đang theo dõi"""
    id: int               # ID duy nhất của đối tượng
    class_name: str       # Loại đối tượng
    position: tuple       # Vị trí hiện tại (x,y)
    velocity: tuple       # Vận tốc (vx, vy)
    predicted_pos: tuple  # Vị trí dự đoán tiếp theo
    last_seen: float     # Thời điểm cuối cùng thấy đối tượng
    tracking_state: int   # Trạng thái theo dõi

class Tracking(QObject):
    # Signals
    object_tracked = Signal(TrackedObject)      # Phát hiện được đối tượng
    object_lost = Signal(int)                   # Mất dấu đối tượng (emit id)
    pickup_ready = Signal(TrackedObject)        # Đối tượng sẵn sàng để gắp
    
    def __init__(self, conveyor_speed=None):
        super().__init__()
        self.tracked_objects = {}  # Dict lưu các đối tượng đang theo dõi
        self.next_id = 0          # ID cho đối tượng tiếp theo
        self.conveyor_speed = conveyor_speed  # Tốc độ băng chuyền
        
        # Kalman filter cho mỗi đối tượng
        self.kalman_filters = {}
        
        # Timer để cập nhật vị trí
        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self._update_tracking)
        self._update_timer.start(50)  # 20Hz
        
    def init_kalman_filter(self):
        """Khởi tạo Kalman filter cho đối tượng mới"""
        kf = cv2.KalmanFilter(4, 2)  # 4 state variables (x,y,vx,vy), 2 measurement variables (x,y)
        kf.measurementMatrix = np.array([[1,0,0,0], [0,1,0,0]], np.float32)
        kf.transitionMatrix = np.array([[1,0,1,0], [0,1,0,1], [0,0,1,0], [0,0,0,1]], np.float32)
        kf.processNoiseCov = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]], np.float32) * 0.03
        return kf
        
    def add_object(self, detected_obj, timestamp):
        """Thêm đối tượng mới vào danh sách theo dõi"""
        obj_id = self.next_id
        self.next_id += 1
        
        # Khởi tạo Kalman filter cho đối tượng
        kf = self.init_kalman_filter()
        self.kalman_filters[obj_id] = kf
        
        # Tạo đối tượng theo dõi mới
        tracked_obj = TrackedObject(
            id=obj_id,
            class_name=detected_obj.class_name,
            position=detected_obj.center,
            velocity=(0,0),
            predicted_pos=detected_obj.center,
            last_seen=timestamp,
            tracking_state=1
        )
        
        self.tracked_objects[obj_id] = tracked_obj
        return tracked_obj
        
    def update_object(self, obj_id, new_pos, timestamp):
        """Cập nhật vị trí của đối tượng"""
        if obj_id not in self.tracked_objects:
            return
            
        obj = self.tracked_objects[obj_id]
        kf = self.kalman_filters[obj_id]
        
        # Cập nhật Kalman filter
        measurement = np.array([[np.float32(new_pos[0])], [np.float32(new_pos[1])]])
        kf.correct(measurement)
        
        # Dự đoán vị trí tiếp theo
        prediction = kf.predict()
        predicted_pos = (prediction[0][0], prediction[1][0])
        
        # Tính vận tốc
        dt = timestamp - obj.last_seen
        if dt > 0:
            velocity = (
                (new_pos[0] - obj.position[0]) / dt,
                (new_pos[1] - obj.position[1]) / dt
            )
        else:
            velocity = obj.velocity
            
        # Cập nhật đối tượng
        obj.position = new_pos
        obj.velocity = velocity
        obj.predicted_pos = predicted_pos
        obj.last_seen = timestamp
        
        self.object_tracked.emit(obj)
        
        # Kiểm tra vị trí để gắp
        if self._check_pickup_position(obj):
            self.pickup_ready.emit(obj)
            
    def _update_tracking(self):
        """Cập nhật định kỳ trạng thái các đối tượng"""
        current_time = time.time()
        
        # Kiểm tra và cập nhật từng đối tượng
        for obj_id in list(self.tracked_objects.keys()):
            obj = self.tracked_objects[obj_id]
            
            # Xóa đối tượng nếu mất dấu quá lâu
            if current_time - obj.last_seen > 1.0:  # timeout 1s
                self.object_lost.emit(obj_id)
                del self.tracked_objects[obj_id]
                del self.kalman_filters[obj_id]
                continue
                
            # Cập nhật vị trí dự đoán
            if self.conveyor_speed:
                # Tính toán vị trí dựa trên tốc độ băng chuyền
                dt = current_time - obj.last_seen
                dx = self.conveyor_speed * dt
                new_pos = (obj.position[0] + dx, obj.position[1])
                self.update_object(obj_id, new_pos, current_time)
                
    def _check_pickup_position(self, obj):
        """Kiểm tra xem đối tượng đã đến vị trí có thể gắp chưa"""
        # Implement your pickup zone check here
        return False
        
    def get_pickup_trajectory(self, obj):
        """Tính toán quỹ đạo gắp tối ưu cho đối tượng đang di chuyển"""
        if obj.id not in self.tracked_objects:
            return None
            
        # Tính thời gian để robot di chuyển đến điểm gắp
        robot_move_time = 0.5  # giả sử
        
        # Dự đoán vị trí của đối tượng sau khoảng thời gian đó
        future_pos = (
            obj.position[0] + obj.velocity[0] * robot_move_time,
            obj.position[1] + obj.velocity[1] * robot_move_time
        )
        
        return future_pos 