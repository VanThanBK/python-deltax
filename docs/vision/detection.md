# Object Detection

The vision system provides multiple object detection methods.

## Color Detection

```python
from deltax.vision import ColorDetector

# Initialize detector
detector = ColorDetector()

# Add color ranges to detect
detector.add_color("red", 
    lower_bgr=(0,0,100),
    upper_bgr=(50,50,255)
)

# Process frame
detections = detector.detect(frame)
for det in detections:
    print(f"Found {det.class_name} at {det.center}")
```

## YOLO Detection

```python
from deltax.vision import YOLODetector

# Initialize YOLO detector
detector = YOLODetector(
    model_path="models/yolov5s.pt",
    confidence=0.5
)

# Process frame
detections = detector.detect(frame)
```

## Detection Results

The `Detection` class contains:
- `class_name`: Object class
- `confidence`: Detection confidence (0-1)
- `bbox`: Bounding box (x,y,w,h)
- `center`: Object center point (x,y)

## Camera Calibration

```python
from deltax.vision import CameraCalibrator

calibrator = CameraCalibrator()

# Calibrate camera intrinsics
calibrator.calibrate_camera(images)

# Calibrate transform
world_points = [(0,0), (100,0), (0,100), (100,100)]
image_points = [(100,100), (200,100), (100,200), (200,200)]
calibrator.calibrate_transform(world_points, image_points)

# Convert coordinates
robot_pos = calibrator.image_to_world(pixel_pos)
``` 