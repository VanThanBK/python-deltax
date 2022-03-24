# Python library to control a robot from 'Delta X Robot'

## exemple

### DeltaX S
```Python
robot = DeltaX(port = "COM4", model=DeltaX.DeltaX_S)
if robot.connect() == True:
    print("connected")
robot.syncPosition()
robot.wait_for_robot_response()
print(robot.position())
robot.syncInput(I=[0, 1], A=[0])
robot.wait_for_robot_response()
print(robot.getDigitalInput(I=[0, 1]))
print(robot.getAnalogInput(A=[0]))
robot.setDO(D=[0,1], value=DeltaX.ON)
robot.wait_for_robot_response()
while 1:
    robot.moveL([20,0,-800])
    robot.moveL([-20,0,-800])
    robot.moveC(offset=[-30,-30], point=[-20, 0])
    robot.wait_for_robot_response()
```

### DeltaX V2
```Python
robot = DeltaX(port = "COM4", model=DeltaX.DeltaX_V2)
if robot.connect() == True:
    print("connected")
robot.syncPosition()
robot.wait_for_robot_response()
print(robot.position())
robot.setEndEffector(name=DeltaX.Vacuum)
robot.wait_for_robot_response()
robot.controlEndEffector(value=DeltaX.ON)
robot.wait_for_robot_response()
while 1:
    robot.moveL([20,0,-270])
    robot.moveL([-20,0,-270])
    robot.moveC(offset=[-30,-30], point=[-20, 0])
    robot.wait_for_robot_response()
```

## list func
    - connect
    - disconnect
    - is_connected
    - sendGcode
    - wait_for_robot_response
    - robot_response
    - syncMotionParameters
    - motionParameters
    - sleep
    - position
    - angle
    - homing
    - syncPosition
    - syncAngle
    - syncInput
    - getDigitalInput
    - getAnalogInput
    - setDO
    - controlEndEffector
    - setEndEffector
    - disableSteppers
    - setAcceleration
    - setStartingAndEndingSpeeds
    - setXYZOffset
    - moveL
    - moveC



![DeltaX S](image.png)
