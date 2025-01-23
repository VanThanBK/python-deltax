from PySide6.QtTest import QTest
from deltax.core import Robot, Encoder, Conveyor, Camera

def test_device_connection(qtbot):
    """Test device connection"""
    # Test robot
    robot = Robot(port="MOCK")
    with qtbot.waitSignal(robot.connected, timeout=1000):
        assert robot.connect()
    assert robot.is_connected()
    
    # Test encoder
    encoder = Encoder(port="MOCK")
    with qtbot.waitSignal(encoder.connected, timeout=1000):
        assert encoder.connect()
    assert encoder.is_connected()
    
    # Test conveyor
    conveyor = Conveyor(port="MOCK")
    with qtbot.waitSignal(conveyor.connected, timeout=1000):
        assert conveyor.connect()
    assert conveyor.is_connected()

def test_camera(qtbot):
    """Test camera"""
    camera = Camera(0)
    
    # Test frame signal
    with qtbot.waitSignal(camera.frame_ready, timeout=1000):
        assert camera.start()
        
    # Get frame
    frame = camera.get_frame()
    assert frame is not None
    
    # Stop camera
    camera.stop() 