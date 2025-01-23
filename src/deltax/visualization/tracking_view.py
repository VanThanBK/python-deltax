from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter, QColor
from ..tracking import ObjectTracker

class TrackingView(QWidget):
    """Visualizes tracked objects"""
    
    def __init__(self, tracker: ObjectTracker, parent=None):
        super().__init__(parent)
        self.tracker = tracker
        self.tracker.object_tracked.connect(self.update)
        
        # Window shows -500 to 500 in x and y
        self.scale = self.width() / 1000.0
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw conveyor
        painter.setPen(Qt.gray)
        y = self.height() / 2
        painter.drawLine(0, y, self.width(), y)
        
        # Draw objects
        painter.setPen(Qt.black)
        painter.setBrush(QColor(0, 150, 0))
        
        for obj in self.tracker._objects:
            # Convert coordinates
            x = (obj.x + 500) * self.scale
            y = (obj.y + 500) * self.scale
            
            # Draw circle
            painter.drawEllipse(x-5, y-5, 10, 10)
            
            # Draw ID
            painter.drawText(x-10, y-10, str(obj.id)) 