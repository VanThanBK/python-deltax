# Error Handling

DeltaX provides comprehensive error handling and recovery mechanisms.

## Error Types

### RobotError
Errors related to robot control:
```python
try:
    robot.move(x, y, z)
except RobotError as e:
    print(f"Robot error: {e}")
```

### CameraError
Camera and vision system errors:
```python
try:
    frame = camera.get_frame()
except CameraError as e:
    print(f"Camera error: {e}")
```

### SystemError
High-level system errors:
```python
try:
    system.start()
except SystemError as e:
    print(f"System error: {e}")
```

## Error Recovery

The framework includes automatic error recovery:

```python
from deltax import DeltaXFramework
from deltax.recovery import ErrorHandler

framework = DeltaXFramework()

# Register custom error handler
@framework.error_handler.register_handler("RobotError")
def handle_robot_error(error):
    print(f"Handling robot error: {error}")
    return framework.reconnect_robot()
```

## Error Levels

- `INFO`: Non-critical, can continue
- `WARNING`: May need attention
- `ERROR`: Needs recovery
- `CRITICAL`: Must stop system

## Logging

Errors are automatically logged:

```python
from deltax.logger import Logger

logger = Logger(__name__)
logger.error("An error occurred", exc_info=True)
```

Log files are stored in:
- `~/.deltax/logs/deltax.log`: Main log file
- `~/.deltax/logs/errors.log`: Error-only log 