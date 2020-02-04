#!/usr/bin/env pybricks-micropython
 
import time
from threading import Thread
 
from pybricks import ev3brick as brick
from pybricks.ev3devices import (ColorSensor, GyroSensor, InfraredSensor,
                                 Motor, TouchSensor, UltrasonicSensor)
from pybricks.parameters import (Align, Button, Color, Direction, ImageFile,
                                 Port, SoundFile, Stop)
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait
while True:
  #write your program hear
  left_M = Motor(Port.C) #C
  right_M = Motor(Port.B)#B
  small_left_m = Motor(Port.A)#A
  '''small_right_m = Motor(Port.D)#D'''
  first_gyro = GyroSensor(Port.S1)#3
  second_gyro = GyroSensor(Port.S4)#2
  first_cls = ColorSensor(Port.S3)#4
  second_cls = ColorSensor(Port.S2)#1
  robot = DriveBase(left_M,right_M,62,95)#62 95
  positionX = 0
  PositionY = 0
  #defs:
  def in_range(every_deviation2,range1,range2):
    if every_deviation2 in range(range1,range2):
      every_deviation2 = 0
    else:
      print("")
  def reset_all():
    first_gyro.reset_angle(0)
    second_gyro.reset_angle(0)
    left_M.reset_angle(0)
    right_M.reset_angle(0)
  
  def run_motor_tank_right (speed):
    right_M.dc(speed)
  
  def run_motor_tank_left(speed):
    left_M.dc(speed)
  
  def accurate_sensor(sensor1,sensor2):
    sensor1 = sensor1.angle()
    sensor2 = sensor2.angle()
    accurate_sensor2 = sensor1 + sensor2
    accurate_sensor2 = accurate_sensor2/2
    return accurate_sensor2
  
  def accurate_cls():
    accurate_cls2 = first_cls.reflection() - second_cls.reflection()
    return accurate_cls2
  
  def PID (speed,angle,kp,ki,kd,last_deviation=0,every_deviation=0):
    ki = ki/100
    accurate_sensor_pid=accurate_sensor(first_gyro,second_gyro)
    deviation=accurate_sensor_pid-angle
    p=kp*deviation
    every_deviation += deviation
    in_range(every_deviation,5,-5)
    i=ki*every_deviation
    d=(deviation-last_deviation)*kd
    pid_angle = p+i+d
    robot.drive(speed,pid_angle)
    last_deviation = deviation
    return last_deviation,every_deviation
  
  def followline(speed,kp,ki,kd,last_notblack=0,every_notblack=0 ):
    ki = ki/100
    notblack = accurate_cls()
    every_notblack += notblack
    in_range(every_notblack,-15,15)
    p = kp*notblack
    i=ki*every_notblack
    d=(notblack-every_notblack)*kd
    pid_angle = p+i+d
    PID(speed,pid_angle,1,1,1)
    last_notblack = notblack
    return last_notblack,every_notblack
  
  def TurnInPlaceRight(Angle):
    PresentAngle = accurate_sensor(first_gyro,second_gyro)
    AdditionAngle = Angle
    EndAngle = PresentAngle + AdditionAngle
    while EndAngle>PresentAngle:
      PresentAngle = accurate_sensor(sensor10,sensor11 )
      UpdatedAdditionalAngle = EndAngle - PresentAngle
      if UpdatedAdditionalAngle<5:
        break
      if AdditionAngle>0:
        robot.drive(10,100)
      else:
        break 
    robot.stop()
  
  def TurnInPlaceLeft(Angle):
    PresentAngle = accurate_sensor(first_gyro,second_gyro)
    AdditionAngle = Angle
    EndAngle = PresentAngle + AdditionAngle
    while EndAngle>PresentAngle:
      PresentAngle = accurate_sensor(sensor10,sensor11 )
      UpdatedAdditionalAngle = EndAngle - PresentAngle
      if UpdatedAdditionalAngle<5:
        break
      if AdditionAngle>0:
        robot.drive(10,100)
      else:
        break 
    robot.stop()
  def check_gyro_drift():
    while True:
      if first_gyro.angle()>0:
        if second_gyro.angle()>0:
          brick.display.text("drift in port 3,2", (60, 50))
        else:
          brick.display.text("drift in port 3",(60, 50))
      elif second_gyro.angle()>0:
        brick.display.text("drift in port 2",(60, 50))
      else:
        brick.display.text("were ready")
  def motor_avarge():
    avarge_motors = left_M.angle()+ right_M.angle()
    avarge_motors = avarge_motors/2
    return avarge_motors
  
  def stop1():
    if Button.CENTER in brick.buttons():
      left_M.stop()
      right_M.stop()
      '''small_left_m.stop()'''
      '''small_right_m.stop()'''
      while True:
        if first_cls.color() == Color.RED:
          while not Button.CENTER in brick.buttons():
            reset_all()
            while motor_avarge()<=1200:
              PID(150,0,5,1,1)
              stop2()
            robot.stop()
            reset_all()
            time.sleep(1)
            while motor_avarge()>=-270:
              PID(-400,0,1,1,1)
              stop2()
            robot.stop()
            while True:
              PID(-500,-210,1,1,1)
              stop2()
        elif Button.LEFT in brick.buttons():
          while not Button.CENTER in brick.buttons():
            reset_all()
            while motor_avarge()<=970:
              PID(200,0,8,1,1)
              stop2()
            robot.stop()
            reset_all()
            while motor_avarge()>=-450:
              PID(-200,0,1,1,1)
              stop2()
            robot.stop()
            reset_all()
            while True:
              PID(-500,-200,1,1,1)
              stop2()
        elif Button.UP in brick.buttons():
          while not Button.CENTER in brick.buttons():
            reset_all()
            while motor_avarge()<=780:
              PID(150,0,8,1,1)
              stop2()
            robot.stop()
            left_M.reset_angle(0)
            right_M.reset_angle(0)
            while motor_avarge()<=470:
              followline(100,1.2,1,1)
              stop2()
            robot.stop()
            time.sleep(0.1)
            first_gyro.reset_angle(0)
            second_gyro.reset_angle(0)
            while first_gyro.angle()<=25:
              robot.drive(10,-100)
              stop2()
            robot.stop()
            time.sleep(0.2)
            reset_all()
            while motor_avarge()<=1000:
              PID(160,0,8,1,1)
              stop2()
            robot.stop()
            time.sleep(0.2)
            left_M.reset_angle(0)
            right_M.reset_angle(0)
            if second_cls.reflection()>=65:
              while motor_avarge()<=500:
                followline(100,0.6,1,1)
                stop2()
              robot.stop()
            elif second_cls.reflection()<=10:
              while motor_avarge()<=460:
                followline(100,0.6,1,1)
                stop2()
              robot.stop()
            else:
              while motor_avarge()<=420:
                PID(100,-4,1,1,1)
                stop2()
              robot.stop()
            time.sleep(0.3)
            left_M.reset_angle(0)
            right_M.reset_angle(0)
            while motor_avarge()>=-1500:
              PID(-300,0,1,1,1)
              stop2()
            robot.stop()
            while first_gyro.angle()>=-17:
              robot.drive(10,100)
              stop2()
            robot.stop()
            reset_all()
            while True:
              PID(-700,0,1,1,1)
              stop2()
        elif Button.DOWN in brick.buttons():
          while not Button.CENTER in brick.buttons():
            first_gyro.reset_angle(0)
            second_gyro.reset_angle(0)
            small_left_m = Motor(Port.A)
            i=0
            while i!= 1500:
              small_left_m.dc(40)
              i+=1
              stop2()
            small_left_m.stop()
            time.sleep(0.5)
            left_M.reset_angle(0)
            right_M.reset_angle(0)
            while motor_avarge()<=740:
              PID(150,0,8,1,1)
              stop2()
            robot.stop()
            time.sleep(0.1)
            left_M.reset_angle(0)
            right_M.reset_angle(0)
            while motor_avarge()<=720:
              followline(100,0.5,1,1)
              stop2()
            robot.stop()
            time.sleep(0.5)
            i=0
            while i!= 2000:
              small_left_m.dc(-40)
              i+=1
              stop2()
            small_left_m.stop()
            reset_all()
            while motor_avarge()<=1000:
              PID(150,0,1,1,1)
              stop2()
            robot.stop()
            time.sleep(1)
            reset_all()
            while True:
              PID(-500,0,1,1,1)
              stop2()
        elif Button.RIGHT in brick.buttons():
          while not Button.CENTER in brick.buttons():
            reset_all()
            while motor_avarge()<=740:
              PID(150,0,1,1,1)
              stop2()
            robot.stop()
            time.sleep(0.1)
            left_M.reset_angle(0)
            right_M.reset_angle(0)
            while motor_avarge()<=870:
              followline(150,0.6,1,1)
              stop2()
            robot.stop()
            reset_all()
            while motor_avarge()<=400:
              PID(150,0,1,1,1)
              stop2()
            robot.stop()
            reset_all()
            while first_gyro.angle()<=100:
              robot.drive(10,-100)
              stop1()
            robot.stop()
            i=0
            while i<=110:
              robot.drive(-100,0)
              i+=1
              stop2()
            robot.stop()
            reset_all()
            while motor_avarge()<=270:
              PID(150,0,1,1,1)
              stop2()
            robot.stop()
            reset_all()
            while first_gyro.angle()<=27:
              robot.drive(10,-100)
              stop2()
            robot.stop()
            reset_all()
            while motor_avarge()<=600:
              followline(100,1,1,1)
              stop2()
            robot.stop()
            reset_all()
            while first_gyro.angle()<=170:
              robot.drive(10,-100)
              stop2()
            robot.stop()
            while motor_avarge()>=-1230:
              robot.drive(-1000,0)
              stop2()
            robot.stop()
            reset_all()
            while True:
              if motor_avarge()>0:
                while motor_avarge()>0:
                  robot.drive(-10,0)
                  stop2()
                robot.stop()
              elif motor_avarge()<0:
                while motor_avarge()<0:
                  robot.drive(-10,0)
                  stop2()
                robot.stop()
                while motor_avarge()>=-1200:
                  robot.drive(-2000,0)
                  stop2()
                robot.stop()
                reset_all()
  def stop2():
    if Button.CENTER in brick.buttons():
      left_M.stop()
      right_M.stop()
      '''small_left_m.stop()'''
      '''small_right_m.stop() '''
      while True:
        if first_cls.color() == Color.RED:
          while not Button.CENTER in brick.buttons():
            reset_all()
            while motor_avarge()<=1200:
              PID(150,0,5,1,1)
              stop1()
            robot.stop()
            reset_all()
            time.sleep(1)
            while motor_avarge()>=-270:
              PID(-400,0,1,1,1)
              stop1()
            robot.stop()
            reset_all()
            while True:
              PID(-500,-210,1,1,1)
              stop1()
        elif Button.LEFT in brick.buttons():
          while not Button.CENTER in brick.buttons():
            reset_all()
            while motor_avarge()<=970:
              PID(200,0,8,1,1)
              stop1()
            robot.stop()
            reset_all()
            while motor_avarge()>=-450:
              PID(-200,0,1,1,1)
              stop1()
            robot.stop()
            reset_all()
            while True:
              PID(-500,-200,1,1,1)
              stop1()
        elif Button.UP in brick.buttons():
          while not Button.CENTER in brick.buttons():
            reset_all()
            while motor_avarge()<=780:
              PID(150,0,8,1,1)
              stop1()
            robot.stop()
            left_M.reset_angle(0)
            right_M.reset_angle(0)
            while motor_avarge()<=470:
              followline(100,1.2,1,1)
              stop1()
            robot.stop()
            time.sleep(0.1)
            first_gyro.reset_angle(0)
            second_gyro.reset_angle(0)
            while first_gyro.angle()<=25:
              robot.drive(10,-100)
              stop1()
            robot.stop()
            reset_all()
            while motor_avarge()<=1000:
              PID(160,0,8,1,1)
              stop1()
            robot.stop()
            time.sleep(0.2)
            left_M.reset_angle(0)
            right_M.reset_angle(0)
            if second_cls.reflection()>=65:
              while motor_avarge()<=500:
                followline(100,0.6,1,1)
                stop1()
              robot.stop()
            elif second_cls.reflection()<=10:
              while motor_avarge()<=460:
                followline(100,0.6,1,1)
                stop1()
              robot.stop()
            else:
              while motor_avarge()<=420:
                PID(100,-4,1,1,1)
                stop1()
              robot.stop()
            time.sleep(0.3)
            left_M.reset_angle(0)
            right_M.reset_angle(0)
            while motor_avarge()>=-1500:
              PID(-300,0,1,1,1)
              stop1()
            robot.stop()
            while first_gyro.angle()>=-17:
              robot.drive(10,100)
              stop1()
            robot.stop()
            reset_all()
            while True:
              PID(-700,0,1,1,1)
              stop1()
            robot.stop()
        elif Button.DOWN in brick.buttons():
          while not Button.CENTER in brick.buttons():
            first_gyro.reset_angle(0)
            second_gyro.reset_angle(0)
            small_left_m = Motor(Port.A)
            i=0
            while i!= 1500:
              small_left_m.dc(40)
              i+=1
              stop1()
            small_left_m.stop()
            time.sleep(0.5)
            left_M.reset_angle(0)
            right_M.reset_angle(0)
            while motor_avarge()<=740:
              PID(150,0,8,1,1)
              stop1()
            robot.stop()
            time.sleep(0.1)
            left_M.reset_angle(0)
            right_M.reset_angle(0)
            while motor_avarge()<=720:
              followline(100,0.5,1,1)
              stop1()
            robot.stop()
            time.sleep(0.5)
            i=0
            while i!= 2000:
              small_left_m.dc(-40)
              i+=1
              stop1()
            small_left_m.stop()
            reset_all()
            while motor_avarge()<=1000:
              PID(150,0,1,1,1)
              stop1()
            robot.stop()
            time.sleep(1)
            reset_all()
            while True:
              PID(-500,0,1,1,1)
              stop1()
        elif Button.RIGHT in brick.buttons():
          while not Button.CENTER in brick.buttons():
            reset_all()
            while motor_avarge()<=740:
              PID(150,0,1,1,1)
              stop1()
            robot.stop()
            time.sleep(0.1)
            left_M.reset_angle(0)
            right_M.reset_angle(0)
            while motor_avarge()<=870:
              followline(150,0.6,1,1)
              stop1()
            robot.stop()
            reset_all()
            while motor_avarge()<=400:
              PID(150,0,1,1,1)
              stop1()
            robot.stop()
            reset_all()
            while first_gyro.angle()<=100:
              robot.drive(10,-100)
              stop1()
            robot.stop()
            i=0
            while i<=110:
              robot.drive(-100,0)
              i+=1
              stop1()
            robot.stop()
            reset_all()
            while motor_avarge()<=270:
              PID(150,0,1,1,1)
              stop1()
            robot.stop()
            reset_all()
            while first_gyro.angle()<=27:
              robot.drive(10,-100)
              stop1()
            robot.stop()
            reset_all()
            while motor_avarge()<=600:
              followline(100,1,1,1)
              stop1()
            robot.stop()
            reset_all()
            while first_gyro.angle()<=170:
              robot.drive(10,-100)
              stop1()
            robot.stop()
            while motor_avarge()>=-1230:
              robot.drive(-1000,0)
              stop1()
            robot.stop()
            reset_all()
            while True:
              if motor_avarge()>0:
                while motor_avarge()>0:
                  robot.drive(-10,0)
                  stop1()
                robot.stop()
              elif motor_avarge()<0:
                while motor_avarge()<0:
                  robot.drive(-10,0)
                  stop1()
                robot.stop()
                while motor_avarge()>=-1300:
                  robot.drive(-2000,0)
                  stop1()
                robot.stop()
                reset_all()
  
  while True:
    if first_cls.color() == Color.RED:
      while not Button.CENTER in brick.buttons():
        reset_all()
        while motor_avarge()<=1400:
          PID(150,0,5,1,1)
          stop1()
        robot.stop()
        reset_all()
        time.sleep(1)
        while motor_avarge()>=-270:
          PID(-400,0,1,1,1)
          stop1()
        robot.stop()
        while True:
          PID(-500,-210,1,1,1)
          stop1()
    elif Button.LEFT in brick.buttons():
      while not Button.CENTER in brick.buttons():
        reset_all()
        while motor_avarge()<=970:
          PID(200,0,8,1,1)
          stop1()
        robot.stop()
        reset_all()
        while motor_avarge()>=-450:
          PID(-200,0,1,1,1)
          stop1()
        robot.stop()
        reset_all()
        while True:
          PID(-500,-200,1,1,1)
          stop1()
    elif Button.UP in brick.buttons():
      while not Button.CENTER in brick.buttons():
        reset_all()
        while motor_avarge()<=780:
          PID(150,0,8,1,1)
          stop1()
        robot.stop()
        left_M.reset_angle(0)
        right_M.reset_angle(0)
        while motor_avarge()<=470:
          followline(100,1.2,1,1)
          stop1()
        robot.stop()
        time.sleep(0.1)
        first_gyro.reset_angle(0)
        second_gyro.reset_angle(0)
        while first_gyro.angle()<=25:
          robot.drive(10,-100)
          stop1()
        robot.stop()
        reset_all()
        while motor_avarge()<=1000:
          PID(160,0,8,1,1)
          stop1()
        robot.stop()
        time.sleep(0.2)
        left_M.reset_angle(0)
        right_M.reset_angle(0)
        if second_cls.reflection()>=65:
          while motor_avarge()<=500:
            followline(100,0.6,1,1)
            stop1()
          robot.stop()
        elif second_cls.reflection()<=10:
          while motor_avarge()<=460:
            followline(100,0.6,1,1)
          robot.stop()
        else:
          while motor_avarge()<=420:
            PID(100,-4,1,1,1)
            stop1()
          robot.stop()
        time.sleep(0.3)
        left_M.reset_angle(0)
        right_M.reset_angle(0)
        while motor_avarge()>=-1500:
          PID(-300,0,1,1,1)
          stop1()
        robot.stop()
        while first_gyro.angle()>=-17:
          robot.drive(10,100)
          stop1()
        robot.stop()
        reset_all()
        while True:
          PID(-700,0,1,1,1)
          stop1()
        robot.stop()
    elif Button.DOWN in brick.buttons():
      while not Button.CENTER in brick.buttons():
        first_gyro.reset_angle(0)
        second_gyro.reset_angle(0)
        small_left_m = Motor(Port.A)
        i=0
        while i!= 1500:
          small_left_m.dc(40)
          i+=1
          stop1()
        small_left_m.stop()
        time.sleep(0.5)
        left_M.reset_angle(0)
        right_M.reset_angle(0)
        while motor_avarge()<=740:
          PID(150,0,8,1,1)
          stop1()
        robot.stop()
        time.sleep(0.1)
        left_M.reset_angle(0)
        right_M.reset_angle(0)
        while motor_avarge()<=720:
          followline(100,0.5,1,1)
          stop1()
        robot.stop()
        time.sleep(0.5)
        i=0
        while i!= 2000:
          small_left_m.dc(-40)
          i+=1
          stop1()
        small_left_m.stop()
        reset_all()
        while motor_avarge()<=1000:
          PID(150,0,1,1,1)
          stop1()
        robot.stop()
        time.sleep(1)
        reset_all()
        while True:
          PID(-500,0,1,1,1)
          stop1()
    elif Button.RIGHT in brick.buttons():
      while not Button.CENTER in brick.buttons():
        reset_all()
        while motor_avarge()<=740:
          PID(150,0,1,1,1)
          stop1()
        robot.stop()
        time.sleep(0.1)
        left_M.reset_angle(0)
        right_M.reset_angle(0)
        while motor_avarge()<=870:
          followline(150,0.6,1,1)
          stop1()
        robot.stop()
        reset_all()
        while motor_avarge()<=400:
          PID(150,0,1,1,1)
          stop1()
        robot.stop()
        reset_all()
        while first_gyro.angle()<=100:
          robot.drive(10,-100)
          stop1()
        robot.stop()
        i=0
        while i<=110:
          robot.drive(-100,0)
          i+=1
          stop1()
        robot.stop()
        reset_all()
        while motor_avarge()<=270:
          PID(150,0,1,1,1)
          stop1()
        robot.stop()
        reset_all()
        while first_gyro.angle()<=27:
          robot.drive(10,-100)
          stop1()
        robot.stop()
        reset_all()
        while motor_avarge()<=600:
          followline(100,1,1,1)
          stop1()
        robot.stop()
        reset_all()
        while first_gyro.angle()<=170:
          robot.drive(10,-100)
          stop1()
        robot.stop()
        while motor_avarge()>=-1230:
          robot.drive(-1000,0)
          stop1()
        robot.stop()
        reset_all()
        while True:
          if motor_avarge()>0:
            while motor_avarge()>0:
              robot.drive(-10,0)
              stop1()
            robot.stop()
          elif motor_avarge()<0:
            while motor_avarge()<0:
              robot.drive(-10,0)
              stop1()
            robot.stop()
            while motor_avarge()>=-1200:
              robot.drive(-2000,0)
              stop1()
            robot.stop()
            reset_all()
