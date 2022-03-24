from deltax import DeltaX

robot = DeltaX(port = "COM4", model=DeltaX.DeltaX_V2)
if robot.connect() == True:
    print("connected")
robot.moveL([20,50,-800])
robot.wait_for_robot_response()
robot.syncPosition()
robot.wait_for_robot_response()
print(robot.position())
robot.moveL([20,20,-800])
robot.moveL([20,10,-800])
robot.wait_for_robot_response()
robot.setDO(D=[0,1], value=DeltaX.ON)
robot.wait_for_robot_response()
robot.syncMotionParameters(axis=DeltaX.AXIS_XYZ)
robot.wait_for_robot_response()
print(robot.motionParameters(axis=DeltaX.AXIS_XYZ))
robot.syncInput(I=[0, 1], A=[0])
robot.wait_for_robot_response()
robot.sendGcode("G0 X10 Y10")
robot.setEndEffector(name=DeltaX.Vacuum)
robot.wait_for_robot_response()
robot.controlEndEffector(dir = DeltaX.CCW, value=DeltaX.ON)
robot.wait_for_robot_response()


print(robot.getDigitalInput(I=[0, 1]))
print(robot.getAnalogInput(A=[0]))

while 1:
    robot.moveL([20,0,-800])
    robot.moveL([-20,0,-800])
    robot.moveC(offset=[-30,-30], point=[-20, 0])
    robot.wait_for_robot_response()

