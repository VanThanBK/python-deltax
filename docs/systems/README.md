# System Integration

DeltaX provides pre-built system implementations for common applications.

## Available Systems

### FeederSystem
For pick & place from feeder to tray:
```python
from deltax import FeederSystem

system = FeederSystem()
system.setup({
    'robot': {'port': 'COM3'},
    'camera': {'id': 0},
    'feeder': {'pin': 1}
})

# Configure tray positions
system.configure_tray([
    (100, 100),
    (100, 200),
    (200, 100),
    (200, 200)
])

system.start()
```

### ConveyorSystem
For conveyor tracking applications:
```python
from deltax import ConveyorSystem

system = ConveyorSystem()
system.setup({
    'robot': {'port': 'COM3'},
    'camera': {'id': 0},
    'conveyor': {
        'encoder_port': 'COM4',
        'speed': 100
    }
})

system.start()
```

### MultiRobotSystem
For multi-robot coordination:
```python
from deltax import MultiRobotSystem

system = MultiRobotSystem()
system.setup({
    'robots': [
        {'port': 'COM3', 'workspace': {...}},
        {'port': 'COM4', 'workspace': {...}}
    ],
    'camera': {'id': 0},
    'conveyors': {
        'input': {'port': 'COM5'},
        'output': {'port': 'COM6'}
    }
})

system.start()
```

## Custom Systems

Create custom systems by inheriting BaseSystem:

```python
from deltax.systems import BaseSystem

class CustomSystem(BaseSystem):
    def setup(self, config: dict):
        # Setup components
        pass
        
    def start(self):
        # Start operation
        pass
``` 