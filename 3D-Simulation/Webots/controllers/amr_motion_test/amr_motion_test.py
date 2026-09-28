"""First physical milestone: independent open-loop wheel commands per AMR."""
from controller import Robot
robot=Robot()
step=int(robot.getBasicTimeStep())
left=robot.getDevice('left_wheel_motor')
right=robot.getDevice('right_wheel_motor')
for motor in (left,right):
    motor.setPosition(float('inf'))
    motor.setVelocity(0)
while robot.step(step)!=-1:
    t=robot.getTime()
    v,w=(0,0)
    if 2<=t<8:v=.30
    elif 8<=t<11:w=.50
    elif 11<=t<14:w=-.50
    left.setVelocity((v-.76*w/2)/.14)
    right.setVelocity((v+.76*w/2)/.14)
