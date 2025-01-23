# DeltaX Framework

[![Tests](https://github.com/username/deltax/workflows/Test/badge.svg)](https://github.com/username/deltax/actions)
[![Documentation](https://github.com/username/deltax/workflows/Documentation/badge.svg)](https://username.github.io/deltax/)
[![PyPI version](https://badge.fury.io/py/deltax.svg)](https://pypi.org/project/deltax/)
[![codecov](https://codecov.io/gh/username/deltax/branch/main/graph/badge.svg)](https://codecov.io/gh/username/deltax)

DeltaX is a comprehensive framework for controlling delta robots and vision systems. It provides high-level abstractions and tools for building pick & place applications.

## Features

- Delta robot control with multiple model support
- Vision system integration with object detection and tracking
- Conveyor tracking and synchronization
- Flexible system configurations
- Error recovery and logging
- Plugin system for extensions

## Installation

```bash
pip install deltax
```

## Quick Start

```python
from deltax import DeltaXFramework, Robot, Camera

# Create framework instance
framework = DeltaXFramework()

# Connect to robot
framework.connect_robot(port="COM3", model=Robot.X2)

# Start camera
framework.start_camera(camera_id=0)

# Start operation
framework.start()
```

## Examples

See the `examples` directory for complete application examples:

- Feeder to tray application
- Conveyor tracking
- Multi-robot system
- Camera calibration

## Documentation

Full documentation is available at [docs/](docs/README.md)

## License

MIT License - see LICENSE file for details
