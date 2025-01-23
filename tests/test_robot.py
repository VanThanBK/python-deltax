import pytest
import numpy as np
from deltax import Robot
from deltax.errors import RobotError
from PySide6.QtTest import QTest

def test_robot_init():
    """Test robot initialization"""
    robot = Robot(port="COM1", model=Robot.X2)
    assert robot.port == "COM1"
    assert robot.model == Robot.X2
    assert not robot._connected

def test_robot_move(mock_robot):
    """Test robot movement"""
    # Test valid move
    assert mock_robot.move(100, 100, -300)
    assert mock_robot._position == [100, 100, -300]
    
    # Test workspace limits
    mock_robot.set_workspace([-200, 200, -200, 200, -400, 0])
    with pytest.raises(RobotError):
        mock_robot.move(300, 0, 0)  # Outside limits

def test_robot_pick_place(mock_robot):
    """Test pick and place operations"""
    # Test pick
    assert mock_robot.pick(100, 100, -300)
    
    # Test place
    assert mock_robot.place(200, 200, -300)

def test_robot_settings(mock_robot):
    """Test robot settings"""
    mock_robot.set_speed(2000)
    assert mock_robot._speed == 2000
    
    mock_robot.set_acceleration(25000)
    assert mock_robot._acceleration == 25000

def test_gcode_execution(mock_robot):
    """Test G-code execution"""
    assert mock_robot.execute_gcode("G1 X100 Y100 Z-300 F2000")
    assert mock_robot._position == [100, 100, -300]
    assert mock_robot._speed == 2000

def test_safety_limits(mock_robot):
    """Test safety limit checks"""
    mock_robot.set_workspace([-200, 200, -200, 200, -400, 0])
    
    # Test valid move
    assert mock_robot.check_limits(100, 100, -300)
    
    # Test invalid move
    assert not mock_robot.check_limits(300, 0, 0) 

def test_robot_io(mock_robot):
    """Test digital I/O control"""
    # Test output control
    assert mock_robot.set_output(Robot.OUTPUT_1, True)
    assert mock_robot._outputs[1] == True
    
    # Test feeder control
    assert mock_robot.control_feeder(True)
    assert mock_robot._outputs[1] == True
    assert mock_robot.control_feeder(False)
    assert mock_robot._outputs[1] == False 

def test_robot_connection(qtbot):
    """Test robot connection"""
    robot = Robot(port="MOCK")
    
    # Connect signals
    with qtbot.waitSignal(robot.connected, timeout=1000):
        assert robot.connect()
        
    assert robot.is_connected()
    
    # Test command
    response = robot.send_command("TEST")
    assert response == "OK"
    
    # Disconnect
    robot.disconnect()
    assert not robot.is_connected() 