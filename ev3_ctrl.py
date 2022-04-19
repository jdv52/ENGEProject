#!/usr/bin/env pybricks-micropython
# -*- coding: utf-8 -*-
#----------------------------------------------------------------------------
# Created By  : Jayson De La Vega
# Created Date: 4/5/22
# version = '1.0'
# ---------------------------------------------------------------------------
""" This program contains a control loop to run the EV3 brick """
# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
ir_sense = InfraredSensor(Port.S3)
RightTank = Motor(Port.A)
LeftTank = Motor(Port.B)


# Write your program here.
ev3.speaker.beep()
while 1:
    if Button.LEFT_UP in ir_sense.buttons(1):
        LeftTank.dc(100)
    elif Button.LEFT_DOWN in ir_sense.buttons(1):
        LeftTank.dc(-100)
    else:
        LeftTank.stop()
    
    if Button.RIGHT_UP in ir_sense.buttons(1):
        RightTank.dc(100)
    elif Button.RIGHT_DOWN in ir_sense.buttons(1):
        RightTank.dc(-100)
    else:
        RightTank.stop()
    print(str(RightTank.angle()) + ', ' + str(LeftTank.angle()))
