from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
motor_izquierdo=Motor(Port.A,Direction.COUNTERCLOCKWISE)
motor_derecho=Motor(Port.D,Direction.CLOCKWISE)

base=DriveBase(motor_izquierdo,motor_derecho,56,200)

hub.imu.reset_heading(0)

while True :
    print(hub.imu.tilt())
    if hub.imu.heading() == 0:
        base.drive(200,0)
    if hub.imu.heading() > 0:
        base.drive(200,-5)
    if hub.imu.heading() < 0:
        base.drive(200,5) 

    if hub.imu.tilt()[0] > 7:     
        base.drive(400,0)
    if hub.imu.tilt()[0] < -7 :     
        base.drive(100,0)
    
