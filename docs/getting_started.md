# Getting Started with DeltaX

## Installation

Install DeltaX using pip:

```bash
pip install deltax
```

For development installation:
```bash
pip install -e .[dev]
```

## Quick Start

### Basic Robot Control

```python
from deltax import Robot

# Connect to robot
robot = Robot(port="COM3")
robot.connect()

# Move robot
robot.move(100, 100, -300)
```

### Vision System

```python
from deltax import Camera
from deltax.vision import ColorDetector

# Setup camera
camera = Camera(0)
camera.start()

# Setup detector
detector = ColorDetector()
detector.add_color("red", (0,0,100), (50,50,255))

# Process frames
while True:
    frame = camera.get_frame()
    detections = detector.detect(frame)
```

### Complete System

```python
from deltax import DeltaXFramework

# Create framework
framework = DeltaXFramework()

# Load configuration
framework.load_config("config.json")

# Start system
framework.start()
```

## Configuration

Create a configuration file `config.json`:

```json
{
    "robot": {
        "port": "COM3",
        "model": "X2",
        "speed": 1000
    },
    "camera": {
        "id": 0,
        "fps": 30
    },
    "vision": {
        "detector": "color",
        "colors": [
            {
                "name": "red",
                "lower": [0,0,100],
                "upper": [50,50,255]
            }
        ]
    }
}
``` 