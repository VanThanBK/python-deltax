# Robot Control

The `Robot` class provides a high-level interface for controlling delta robots.

## Basic Usage

```python
from deltax import Robot

# Initialize robot
robot = Robot(port="COM3", model=Robot.X2)

# Connect to robot
robot.connect()

# Move robot
robot.move(x=100, y=100, z=-300)

# Execute pick & place
robot.pick(100, 100, -300)
robot.place(200, 200, -300)
```

## Configuration

### Robot Models
- `X1`: Basic model
- `X2`: Standard model
- `X3`: Advanced model

### Parameters
- `speed`: Movement speed (mm/s)
- `acceleration`: Movement acceleration (mm/s²)
- `workspace`: Robot workspace limits [xmin,xmax,ymin,ymax,zmin,zmax]

## Methods

### Movement Control

#### move(x: float, y: float, z: float, speed: Optional[float] = None) -> bool
Move robot to absolute position.

Parameters:
- `x, y, z`: Target position in mm
- `speed`: Optional movement speed override

Returns:
- `bool`: True if movement successful

#### pick(x: float, y: float, z: float, speed: Optional[float] = None) -> bool
Execute pick motion sequence.

#### place(x: float, y: float, z: float, speed: Optional[float] = None) -> bool  
Execute place motion sequence.

### Configuration

#### set_speed(speed: float)
Set robot movement speed.

#### set_acceleration(accel: float)
Set robot acceleration.

#### set_workspace(limits: List[float])
Set robot workspace limits.

### Status

#### get_position() -> List[float]
Get current robot position.

#### in_workspace(x: float, y: float, z: float) -> bool
Check if position is within workspace limits.

## Error Handling

The robot control system includes comprehensive error handling:

```python
from deltax.errors import RobotError

try:
    robot.move(x=100, y=100, z=-300)
except RobotError as e:
    print(f"Robot error: {e}")
```

Common errors:
- Connection failures
- Movement errors
- Workspace violations
- Hardware errors 